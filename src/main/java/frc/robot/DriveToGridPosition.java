// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.Pair;
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
import frc.robot.GridPositionHandler.EntranceCheckpoint;
import frc.robot.GridPositionHandler.LogicalGridLocation;
import frc.robot.GridPositionHandler.PhysicalGridLocation;
import frc.robot.commands.FollowPathOnTheFly;
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

  private static double drive_slow_test_vel = 0.5;
  private static double angular_slow_test_vel = 0.2;

  private static PathConstraints constraints =
      new PathConstraints(
          DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
          DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

  public DriveToGridPosition(
      Drivetrain drivetrain, Intake intake, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.controller = controller;
  }

  public Command driveToGridPosAndScore(GridPositions gridPos) {

    // var path1 =
    // drivetrain.generateOnTheFlyTrajectory(SelectedEntrance.BOTTOM.insideCheckpointPos);

    // Logger.getInstance().recordOutput("Odometry/path1", path1.getInitialState().toString());

    // Logger.getInstance()
    //     .recordOutput(
    //         "Odometry/path2",
    //         RobotOdometry.getInstance().getPoseEstimator().getEstimatedPosition().toString());

    return new SequentialCommandGroup(
        // new FollowPathOnTheFly(
        //     // TODO: combine this with the logic for determining which entrance to use
        //     // for testing use bottom entrance bc thats whats set up in the portable
        //     SelectedEntrance.BOTTOM.insideCheckpointPos,
        //     drivetrain,
        //     drive_slow_test_vel,
        //     angular_slow_test_vel),
        new FollowPathOnTheFly(
            gridPos.lineUpPos, drivetrain, drive_slow_test_vel, angular_slow_test_vel),
        new FollowPathOnTheFly(
            gridPos.scorePos, drivetrain, drive_slow_test_vel, angular_slow_test_vel),
        Commands.run(() -> intake.extend(), intake),
        Commands.waitSeconds(1),
        Commands.run(() -> intake.retract(), intake));
  }

  public Command testSequence(GridPositions gridpos) {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(SelectedEntrance.BOTTOM.insideCheckpointPos);
    points.add(gridpos.lineUpPos);
    points.add(gridpos.scorePos);

    return new SequentialCommandGroup(
        new FollowPathOnTheFly(points, drivetrain, drive_slow_test_vel, angular_slow_test_vel));
  }

  public Command testGenerateAndFollow(GridPositions gridPos) {
    ArrayList<PathPoint> pointsEntry = new ArrayList<PathPoint>();

    pointsEntry.add(
        new PathPoint(
            SelectedEntrance.BOTTOM.insideCheckpointPos.getTranslation(),
            Rotation2d.fromDegrees(180)));

    pointsEntry.add(new PathPoint(gridPos.lineUpPos.getTranslation(), Rotation2d.fromDegrees(180)));

    pointsEntry.add(new PathPoint(gridPos.scorePos.getTranslation(), Rotation2d.fromDegrees(180)));

    // ArrayList<Pose2d> pointsExit = new ArrayList<Pose2d>();
    // pointsExit.add(gridPos.lineUpPos);
    // pointsExit.add(SelectedEntrance.BOTTOM.insideCheckpointPos);

    return new SequentialCommandGroup(
        new GenerateAndFollowPath(drivetrain, pointsEntry, constraints, true)

        // new GenerateAndFollowPath(drivetrain, pointsExit, constraints)
        );
  }

  public Command testAllianceFlip(TestPos pos) {
    ArrayList<PathPoint> pointsEntry = new ArrayList<PathPoint>();

    // pointsEntry.add(
    //     new PathPoint(
    //         SelectedEntrance.BOTTOM.insideCheckpointPos.getTranslation(),
    //         Rotation2d.fromDegrees(180)));

    if (DriverStation.getAlliance() == Alliance.Red) {
      pos.flipForRed();
    }

    pointsEntry.add(
        new PathPoint(pos.lineUpPoint.getFirst().getTranslation(), pos.lineUpPoint.getSecond()));

    pointsEntry.add(
        new PathPoint(pos.scorePoint.getFirst().getTranslation(), pos.scorePoint.getSecond()));

    // ArrayList<Pose2d> pointsExit = new ArrayList<Pose2d>();
    // pointsExit.add(gridPos.lineUpPos);
    // pointsExit.add(SelectedEntrance.BOTTOM.insideCheckpointPos);

    return new SequentialCommandGroup(
        new GenerateAndFollowPath(drivetrain, pointsEntry, constraints, true)

        // new GenerateAndFollowPath(drivetrain, pointsExit, constraints)
        );
  }

  public Command testLogicalBay(LogicalGridLocation logicalBay) {
    Alliance alliance = DriverStation.getAlliance();
    PhysicalGridLocation physicalBay =
        GridPositionHandler.getPhysicalLocationForLogicalBay(logicalBay, alliance);

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/physical_bay", physicalBay.name());

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();

    EntranceCheckpoint entranceCheckpoint =
        GridPositionHandler.getEntrance(drivetrain.getPose(), alliance);

    points.add(
        new PathPoint(
            entranceCheckpoint.location.pose.getTranslation(),
            entranceCheckpoint.location.heading,
            entranceCheckpoint.location.pose.getRotation()));

    points.add(
        new PathPoint(
            physicalBay.lineup.pose.getTranslation(),
            physicalBay.lineup.heading,
            physicalBay.lineup.pose.getRotation()));

    points.add(
        new PathPoint(
            physicalBay.score.pose.getTranslation(),
            physicalBay.score.heading,
            physicalBay.score.pose.getRotation()));

    // for (int i = 0; i < points.size(); i++) {
    // }

    return new SequentialCommandGroup(
        new GenerateAndFollowPath(drivetrain, points, constraints, false));
  }

  public Command driveToCommunityCheckPointBasedOnPos() {
    /**
     * Bottom entrance constraints: x < 4.65 0.01 < y < 1.4
     *
     * <p>Top entrance constraints: x < 4.65 4.1 < y < 5.10
     */
    var selectedEntrance = SelectedEntrance.UNKNOWN;

    var currentPos = drivetrain.getPose();

    // too far away
    if (currentPos.getX() > 4.65) {
      return errorRumbleControllerCommand();
    }

    // too far up or down OR in between the two entrances i.e in front of the pad
    if (currentPos.getY() > 5.10
        || currentPos.getY() < 0.01
        || (currentPos.getY() > 1.4 && currentPos.getY() < 4.1)) {
      return errorRumbleControllerCommand();
    }

    // in the opening of the bottom entrance
    if (currentPos.getY() > 0.01 && currentPos.getY() < 1.4) {
      selectedEntrance = SelectedEntrance.BOTTOM;
    }

    // in the opening of the top entrance
    if (currentPos.getY() > 4.1 && currentPos.getY() < 5.10) {
      selectedEntrance = SelectedEntrance.TOP;
    }

    // if it slipped through everything somehow a catch all safety case
    if (selectedEntrance == SelectedEntrance.UNKNOWN) {
      return errorRumbleControllerCommand();
    }

    return new SequentialCommandGroup(
        new FollowPathOnTheFly(
            selectedEntrance.insideCheckpointPos,
            drivetrain,
            drive_slow_test_vel,
            angular_slow_test_vel));
  }

  // replace with better implementation of controller rumble
  public Command errorRumbleControllerCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
        Commands.waitSeconds(0.5),
        new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  public enum GridPositions {
    // GRID_8_LINEUP(2.0, 1.05),
    // GRID_8_SCORE(1.8, 1.05, 180),
    GRID_7(
        new Pose2d(2.0, 1.6, Rotation2d.fromDegrees(180)),
        new Pose2d(1.8, 1.6, Rotation2d.fromDegrees(180))),

    GRID_8(
        new Pose2d(2.0, 1.05, Rotation2d.fromDegrees(180)),
        new Pose2d(1.8, 1.05, Rotation2d.fromDegrees(180))),

    Grid_9(
        new Pose2d(2.0, 0.48, Rotation2d.fromDegrees(180)),
        new Pose2d(1.8, 0.48, Rotation2d.fromDegrees(180)));

    public final Pose2d lineUpPos;
    public final Pose2d scorePos;

    private GridPositions(Pose2d lineUpPos, Pose2d scorePos) {
      this.lineUpPos = lineUpPos;
      this.scorePos = scorePos;
    }
  }

  public enum TestPos {
    GRID_8(
        new Pair<Pose2d, Rotation2d>(
            new Pose2d(2.0, 1.05, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
        new Pair<Pose2d, Rotation2d>(
            new Pose2d(1.8, 1.05, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)));

    private Pair<Pose2d, Rotation2d> lineUpPoint;
    private Pair<Pose2d, Rotation2d> scorePoint;

    private TestPos(Pair<Pose2d, Rotation2d> lineUpPoint, Pair<Pose2d, Rotation2d> scorePoint) {
      this.lineUpPoint = lineUpPoint;
      this.scorePoint = scorePoint;
    }

    public void flipForRed() {
      lineUpPoint =
          new Pair<Pose2d, Rotation2d>(
              new Pose2d(
                  16.5 - lineUpPoint.getFirst().getX(),
                  lineUpPoint.getFirst().getY(),
                  lineUpPoint.getFirst().getRotation()),
              new Rotation2d(-lineUpPoint.getSecond().getCos(), lineUpPoint.getSecond().getSin()));

      scorePoint =
          new Pair<Pose2d, Rotation2d>(
              new Pose2d(
                  16.5 - scorePoint.getFirst().getX(),
                  scorePoint.getFirst().getY(),
                  scorePoint.getFirst().getRotation()),
              scorePoint.getSecond().unaryMinus());
    }
  }

  public static enum SelectedEntrance {
    TOP(new Pose2d(2.17, 4.64, Rotation2d.fromDegrees(0))),
    BOTTOM(new Pose2d(2.17, 0.7, Rotation2d.fromDegrees(0))),
    UNKNOWN(new Pose2d());

    public final Pose2d insideCheckpointPos;

    private SelectedEntrance(Pose2d pos) {
      insideCheckpointPos = pos;
    }
  }
}
