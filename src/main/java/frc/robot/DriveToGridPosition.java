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
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.GridPositionHandler.EntranceCheckpoint;
import frc.lib.team2930.driverassist.GridPositionHandler.LogicalGridLocation;
import frc.lib.team2930.driverassist.GridPositionHandler.PhysicalGridLocation;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
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
          DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
          DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

  public DriveToGridPosition(
      Drivetrain drivetrain, Intake intake, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.controller = controller;
  }

  public Command testLogicalBay(LogicalGridLocation logicalBay) {
    // TODO: clean up this file
    // TODO: skip first checkpoint if inside community
    // TODO: just flip blue bay locations when making red bay locations
    // TODO: if distance between start and first point is small then dont write logic of heading
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

    EntranceCheckpoint entranceCheckpoint =
        GridPositionHandler.getEntrance(drivetrain.getPose(), alliance);

    if (entranceCheckpoint == EntranceCheckpoint.ERROR) {
      return errorRumbleControllerCommand();
    }

    Pose2d currentPose = drivetrain.getPose();

    // if (!(currentPose.getX() < entranceCheckpoint.location.pose.getX())) {
    //   points.add(
    //       new PathPoint(
    //           entranceCheckpoint.location.pose.getTranslation(),
    //           entranceCheckpoint.location.heading,
    //           entranceCheckpoint.location.pose.getRotation()));
    // }

    // points.add(
    //     new PathPoint(
    //         entranceCheckpoint.location.pose.getTranslation(),
    //         entranceCheckpoint.location.heading,
    //         entranceCheckpoint.location.pose.getRotation()));

    Pose2d firstPose = null;

    Pose2d poseBeforeLineup = null;

    var AllianceCommunityBox = GridPositionHandler.getCommunityBoxForAlliance(alliance);

    if (!AllianceCommunityBox.insideBox(currentPose.getTranslation())) {
      for (PoseAndHeading checkPoint : entranceCheckpoint.getOrderOutsideIn()) {
        if (currentPose.getX() > checkPoint.pose.getX()) {
          points.add(EntranceCheckpoint.toPathPoint(checkPoint));
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
