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
import frc.lib.team2930.driverassist.LogicalGridLocation;
import frc.lib.team2930.driverassist.PhysicalGridLocation;
import frc.robot.commands.GenerateAndFollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class DriveToGridPosition {
  private Drivetrain drivetrain;
  private Intake intake;

  private CommandXboxController controller;

  private static PathConstraints constraints =
      new PathConstraints(
          DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.75,
          DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.75);

  public DriveToGridPosition(
      Drivetrain drivetrain, Intake intake, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.controller = controller;
  }

  public Command testLogicalBay(LogicalGridLocation logicalBay) {
    // TODO optimize to allow paths from in front of charging pad (just add more checkpoints i
    // think)
    // TODO define all the bounding boxes for where auto is allowed to start from
    Alliance alliance = DriverStation.getAlliance();

    // FIXME: ADD THIS BACK
    // boolean validStart = GridPositionHandler.isValidPointToStart(drivetrain.getPose(), alliance);

    // if (!validStart) {
    //   Logger.getInstance().recordOutput("DriverAssist/GridPosition/valid_start", false);
    //   return errorRumbleControllerCommand();
    // }

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/valid_start", true);

    PhysicalGridLocation physicalBay =
        GridPositionHandler.getPhysicalLocationForLogicalBay(logicalBay, alliance);

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/physical_bay", physicalBay.name());

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();

    EntranceCheckpoints entranceCheckpoints =
        GridPositionHandler.getEntrance(drivetrain.getPose(), alliance);

    if (entranceCheckpoints == EntranceCheckpoints.ERROR) {
      return errorRumbleControllerCommand();
    }

    Pose2d currentPose = drivetrain.getPose();

    // used to optimize heading of the first state
    Pose2d firstPose = null;

    // used to optimize heading of the lineup position state
    Pose2d poseBeforeLineup = null;

    var AllianceCommunityBox = GridPositionHandler.getSkipCheckpointBoxForAlliance(alliance);

    // if its inside the alliance community ignore the checkpoints
    if (!AllianceCommunityBox.insideBox(currentPose.getTranslation())) {

      for (PoseAndHeading checkPoint : entranceCheckpoints.getOrderOutsideIn()) {
        // this adds the checkpoints to the list of points if their position comes in between the
        // current position and the target
        if (currentPose.getX() > checkPoint.pose.getX()) {
          points.add(EntranceCheckpoints.toPathPoint(checkPoint));
          if (firstPose == null) {
            firstPose = checkPoint.pose;
          }
          if (poseBeforeLineup == null) {
            poseBeforeLineup = checkPoint.pose;
          }
        }
      }
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

    // for (int i = 0; i < points.size(); i++) {
    // }

    return new SequentialCommandGroup(
        new GenerateAndFollowPath(drivetrain, points, constraints, firstPose, false));
  }

  // replace with better implementation of controller rumble
  public Command errorRumbleControllerCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
        Commands.waitSeconds(0.5),
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }
}
