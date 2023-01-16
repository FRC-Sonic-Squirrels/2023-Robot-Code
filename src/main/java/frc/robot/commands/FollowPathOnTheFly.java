// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowPathOnTheFly extends FollowPath {
  /** Creates a new DriveToSpecificPose. */

  /**
   * Most of the logic is handled by the super class, just generate a trajectory before passing it
   * to the super class
   *
   * @param targetPose
   * @param drivetrain
   */
  public FollowPathOnTheFly(Pose2d targetPose, Drivetrain drivetrain) {
    super(drivetrain.generateOnTheFlyTrajectory(targetPose), drivetrain, false);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public FollowPathOnTheFly(
      Pose2d targetPose,
      Drivetrain drivetrain,
      double driveVelocityConstraint,
      double angularVelocityConstant) {
    super(
        drivetrain.generateOnTheFlyTrajectory(
            targetPose, driveVelocityConstraint, angularVelocityConstant),
        drivetrain,
        false);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // potentially want to not flip based on alliance, needs testing
  public FollowPathOnTheFly(
      Pose2d targetPose,
      Drivetrain drivetrain,
      double driveVelocityConstraint,
      double angularVelocityConstant,
      boolean useAllianceColor) {
    super(
        drivetrain.generateOnTheFlyTrajectory(
            targetPose, driveVelocityConstraint, angularVelocityConstant),
        drivetrain,
        false,
        useAllianceColor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public enum CommonFieldPoses {
    GRID1_SCORING(0.0, 0.0, 0.0),
    PAYLOAD_RIGHT_PICKUP(0.0, 0.0, 0.0);

    public final Pose2d pose;

    private CommonFieldPoses(double x, double y, double rotationDegrees) {
      pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
  }
}
