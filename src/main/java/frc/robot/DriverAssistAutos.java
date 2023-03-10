// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.driverassist.EntranceCheckpoints;
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import frc.lib.team2930.driverassist.HumanLoadingStationHandler;
import frc.lib.team2930.driverassist.HumanLoadingStationHandler.LoadingStationLocation;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import frc.lib.team2930.driverassist.PhysicalGridLocation;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.GenerateAndFollowPath;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class DriverAssistAutos {
  private Drivetrain drivetrain;
  private Intake intake;
  private Elevator elevator;
  private Stinger stinger;

  private CommandXboxController controller;

  private static PathConstraints constraints =
      new PathConstraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.75,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.75);

  public DriverAssistAutos(
      Drivetrain drivetrain,
      Intake intake,
      Elevator elevator,
      Stinger stinger,
      CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.elevator = elevator;
    this.stinger = stinger;
    this.controller = controller;
  }

  public Command driveToLogicalBayClosestEntrance(
      LogicalGridLocation logicalBay, SequentialCommandGroup scoringSequence) {

    var currentPose = drivetrain.getPose();
    var alliance = DriverStation.getAlliance();

    var closestSide = GridPositionHandler.getClosestEntranceSide(currentPose, alliance);

    if (closestSide == null) {
      return errorRumbleControllerCommand();
    }

    return driveToLogicalBaySpecificEntrance(logicalBay, closestSide, scoringSequence);
  }

  public Command driveToLogicalBaySpecificEntrance(
      LogicalGridLocation logicalBay,
      GridPositionHandler.DesiredGridEntrance entranceSide,
      Command scoringSequence) {

    // TODO define all the bounding boxes for where auto is allowed to start from need to do this
    // for human player station

    // TODO obstacle avoidance boxes are not mirrored for red alliance
    Alliance alliance = DriverStation.getAlliance();

    // FIXME: ADD THIS BACK
    boolean validStart = GridPositionHandler.isValidPointToStart(drivetrain.getPose(), alliance);

    if (!validStart) {
      Logger.getInstance().recordOutput("DriverAssist/GridPosition/valid_start", false);
      return errorRumbleControllerCommand();
    }

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/valid_start", true);

    PhysicalGridLocation physicalBay =
        GridPositionHandler.getPhysicalLocationForLogicalBay(logicalBay, alliance);

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/physical_bay", physicalBay.name());

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();

    EntranceCheckpoints entranceCheckpoints =
        GridPositionHandler.getEntranceCheckpointsSpecificSide(
            drivetrain.getPose(), entranceSide, alliance);

    if (entranceCheckpoints == EntranceCheckpoints.ERROR) {
      return errorRumbleControllerCommand();
    }

    Pose2d currentPose = drivetrain.getPose();

    // used to optimize heading of the first state
    Pose2d firstPose = null;

    // used to optimize heading of the lineup position state
    Pose2d poseBeforeLineup = null;

    boolean shouldSkipEntranceCheckpoints =
        GridPositionHandler.shouldSkipEntranceCheckpoints(currentPose.getTranslation(), alliance);

    // FIXME
    // if its inside the alliance community ignore the checkpoints
    if (!shouldSkipEntranceCheckpoints) {

      for (PoseAndHeading checkPoint : entranceCheckpoints.getOrderOutsideIn()) {
        // this adds the checkpoints to the list of points if their position comes in between the
        // current position and the target
        if (GridPositionHandler.shouldFollowEntranceCheckpoint(
            currentPose, checkPoint.pose, alliance)) {
          // FIX ME
          // points.add(EntranceCheckpoints.toPathPoint(checkPoint));
          points.add(
              new PathPoint(
                  checkPoint.pose.getTranslation(),
                  checkPoint.heading,
                  checkPoint.pose.getRotation()));
          if (firstPose == null) {
            firstPose = checkPoint.pose;
          }
          if (poseBeforeLineup == null) {
            poseBeforeLineup = checkPoint.pose;
          }
        }
      }

      // var checkpoints = entranceCheckpoints.getOrderOutsideIn();

      // Logger.getInstance()
      //     .recordOutput("Odometry/usedCheckpoints/rawCheckpoint size", checkpoints.length);

      // List<PoseAndHeading> usedCheckpoints = new ArrayList<>();
      // for (int i = 0; i < checkpoints.length; i++) {

      //   Logger.getInstance().recordOutput("Odometry/usedCheckpoints/i", i);
      //   if (currentPose.getX() < checkpoints[i].pose.getX()) {
      //     continue;
      //   }

      //   Logger.getInstance().recordOutput("Odometry/usedCheckpoints/" + i, checkpoints[i].pose);

      //   usedCheckpoints.add(checkpoints[i]);
      // }

      // var initialRotation = currentPose.getRotation();
      // var finalRotation = physicalBay.score.pose.getRotation();

      // var deltaRotation = finalRotation.minus(initialRotation);

      // double increment = deltaRotation.getRadians() / usedCheckpoints.size();

      // double currentRotationSetPoint = currentPose.getRotation().getRadians();

      // Logger.getInstance()
      //     .recordOutput("Odometry/checkpoint/initailRotation", initialRotation.getDegrees());

      // Logger.getInstance()
      //     .recordOutput("Odometry/checkpoint/finalRotation", finalRotation.getDegrees());

      // Logger.getInstance().recordOutput("Odometry/checkpoint/increment",
      // Math.toDegrees(increment));

      // for (int i = 0; i < usedCheckpoints.size(); i++) {
      //   var poseAndHeading = usedCheckpoints.get(i);
      //   Logger.getInstance()
      //       .recordOutput(
      //           "Odometry/checkpoint/currentRotationsetPoint/" + i,
      //           new Pose2d(
      //               poseAndHeading.pose.getTranslation(),
      //               Rotation2d.fromRadians(currentRotationSetPoint)));

      //   points.add(
      //       new PathPoint(
      //           poseAndHeading.pose.getTranslation(),
      //           poseAndHeading.heading,
      //           Rotation2d.fromRadians(currentRotationSetPoint)));

      //   if (firstPose == null) {
      //     firstPose = poseAndHeading.pose;
      //   }

      //   currentRotationSetPoint += increment;
      // }
    }

    if (firstPose == null) {
      firstPose = physicalBay.lineup.pose;
    }

    if (poseBeforeLineup == null) {
      poseBeforeLineup = currentPose;
    }

    var lineupHeading =
        Math.atan2(
            physicalBay.lineup.pose.getY() - poseBeforeLineup.getY(),
            physicalBay.lineup.pose.getX() - poseBeforeLineup.getX());

    // FIXME
    points.add(
        new PathPoint(
            physicalBay.lineup.pose.getTranslation(),
            new Rotation2d(lineupHeading),
            physicalBay.lineup.pose.getRotation()));

    points.add(
        new PathPoint(
            physicalBay.score.pose.getTranslation(),
            physicalBay.score.heading,
            physicalBay.score.pose.getRotation()));

    // Should skip checkpoints part is for not using our current velocity as our heading if we are
    // inside the community
    // the problem with using the velocity is sometimes it generates a path that goes through the
    // grid which would lead to the robot crashing irl
    return new SequentialCommandGroup(
        new GenerateAndFollowPath(
            drivetrain, points, constraints, firstPose, !shouldSkipEntranceCheckpoints),
        // Commands.runOnce(() -> drivetrain.drive(0, 0, 0), drivetrain),
        scoringSequence);
  }

  public Command humanPlayerStation(
      LoadingStationLocation location, Command pickupSequence, Command retractSequence) {

    // TODO if past last checkpoint then drive to last checkpoint raise elevator and then go to
    // final pose
    Alliance alliance = DriverStation.getAlliance();

    var currentPose = drivetrain.getPose();

    if (!HumanLoadingStationHandler.isValidPointToStart(currentPose, alliance)) {
      return errorRumbleControllerCommand();
    }

    ArrayList<PathPoint> pointsToFollow = new ArrayList<>();

    Pose2d firstPose = null;

    // get checkpoint sequence
    var rawSequence =
        HumanLoadingStationHandler.getCheckpointSequenceForAlliance(location, alliance);

    // find checkpoints we care about

    var checkpointsToFollow =
        HumanLoadingStationHandler.checkpointsToFollow(currentPose, rawSequence, alliance);

    for (int i = 0; i < checkpointsToFollow.length - 1; i++) {
      Logger.getInstance()
          .recordOutput(
              "DriverAssist/humanPlayer/checkpointsToFollow/" + i, checkpointsToFollow[i].pose);
    }

    if (!(checkpointsToFollow.length == 0)) {
      for (PoseAndHeading poseAndHeading : checkpointsToFollow) {
        pointsToFollow.add(
            new PathPoint(
                poseAndHeading.pose.getTranslation(),
                poseAndHeading.heading,
                poseAndHeading.pose.getRotation()));

        if (firstPose == null) {
          firstPose = poseAndHeading.pose;
        }
      }
    } else {
      var lastCheckpoint = rawSequence[rawSequence.length - 1];

      pointsToFollow.add(
          new PathPoint(
              lastCheckpoint.pose.getTranslation(),
              lastCheckpoint.heading,
              lastCheckpoint.pose.getRotation()));
    }

    var finalPose =
        HumanLoadingStationHandler.getFinalPoseForLocationAndAlliance(location, alliance);

    // FIXME prolly dont need this
    if (firstPose == null) {
      firstPose = finalPose.pose;
    }

    ArrayList<PathPoint> secondPathPoints = new ArrayList<>();
    secondPathPoints.add(
        new PathPoint(
            finalPose.pose.getTranslation(), finalPose.heading, finalPose.pose.getRotation()));

    List<PathPoint> retractingPathPoints = new ArrayList<>();

    var lastCheckpoint = rawSequence[rawSequence.length - 1];

    retractingPathPoints.add(
        new PathPoint(
            lastCheckpoint.pose.getTranslation(),
            lastCheckpoint.heading,
            lastCheckpoint.pose.getRotation()));

    // retractingPathPoints.add(
    //     new PathPoint(
    //         rawSequence[rawSequence.length - 1].pose.getTranslation(),
    //         Rotation2d.fromDegrees(180),
    //         Rotation2d.fromDegrees(0)));

    // then go to final score position

    // SequentialCommandGroup returnCommand;
    // var lastHalf =
    //     new SequentialCommandGroup(
    //         // new GenerateAndFollowPath(drivetrain, pointsToFollow, constraints, firstPose,
    // false),
    //         // extend elevator
    //         // might be better to parrellel a slow path with a extension
    //         // rather than a fast path that stops and then extends
    //         Commands.waitSeconds(0.5),
    //         new GenerateAndFollowPath(
    //             drivetrain, secondPathPoints, constraints, finalPose.pose, false));

    // if (!(checkpointsToFollow.length == 0)) {
    //   returnCommand =
    //       new SequentialCommandGroup(
    //           new GenerateAndFollowPath(drivetrain, pointsToFollow, constraints, firstPose,
    // false),
    //           lastHalf);
    // } else {
    //   returnCommand = new SequentialCommandGroup(lastHalf);
    // }

    return new SequentialCommandGroup(
        new GenerateAndFollowPath(drivetrain, pointsToFollow, constraints, firstPose, true),
        // extend elevator
        // might be better to parrellel a slow path with a extension
        // rather than a fast path that stops and then \
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0), drivetrain),
        pickupSequence,
        new GenerateAndFollowPath(drivetrain, secondPathPoints, constraints, finalPose.pose, false),
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0), drivetrain),
        Commands.waitSeconds(0.4),
        new GenerateAndFollowPath(
            drivetrain,
            retractingPathPoints,
            constraints,
            rawSequence[rawSequence.length - 1].pose,
            false),
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0), drivetrain),
        retractSequence);
  }

  public Command getScoringSequenceForGridPositionAuto() {
    var desiredBay = RobotState.getInstance().getDesiredLogicalGrid();

    Command mechanismCommand;
    Command intakeCommand;

    if (desiredBay == LogicalGridLocation.LOGICAL_BAY_2
        || desiredBay == LogicalGridLocation.LOGICAL_BAY_5
        || desiredBay == LogicalGridLocation.LOGICAL_BAY_8) {

      mechanismCommand = MechanismPositions.scoreCubeHighPosition(elevator, stinger);
      intakeCommand = new IntakeScoreCube(intake);
    } else {
      mechanismCommand = MechanismPositions.scoreConeHighPosition(elevator, stinger);
      intakeCommand = new IntakeScoreCone(intake);
    }

    return mechanismCommand
        .andThen(intakeCommand)
        .andThen(Commands.waitSeconds(0.4))
        .andThen(new IntakeStop(intake))
        .andThen(MechanismPositions.stowPosition(elevator, stinger));
  }

  public Command getPickUpSequenceForHumanPlayerStation() {
    Command intakeCommand;

    var desiredGamePiece = RobotState.getInstance().getDesiredGamePiece();

    if (desiredGamePiece == GamePiece.CONE) {
      intakeCommand = new IntakeGrabCone(intake);
    } else {
      intakeCommand = new IntakeGrabCube(intake);
    }

    return intakeCommand.andThen(MechanismPositions.substationPickupPosition(elevator));
  }

  public Command getRetractSequenceForHumanPlayerStation() {
    return new IntakeStop(intake).andThen(MechanismPositions.stowPosition(elevator, stinger));
  }

  // replace with better implementation of controller rumble
  public Command errorRumbleControllerCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
        Commands.waitSeconds(0.5),
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }
}
