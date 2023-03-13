// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

public class SnapToGrid extends CommandBase {
  /** Creates a new snapToGrid. */
  private final Drivetrain drive;

  private static final Pose2d[] bluePos = {
    new Pose2d(1.88, 0.47, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 1.05, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 1.61, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 2.18, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 2.74, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 3.32, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 3.87, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 4.41, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 4.99, new Rotation2d(Math.toRadians(180)))
  }; // CONE

  private static final Pose2d[] redPos = {
    new Pose2d(14.69, 0.47, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 1.05, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 1.61, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 2.18, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 2.74, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 3.32, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 3.87, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 4.41, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 4.99, new Rotation2d(Math.toRadians(0)))
  }; // CONE

  private double xVel = 0;
  private double yVel = 0;

  private Pose2d targetPose = new Pose2d(1000.0, 1000.0, new Rotation2d(0.0));

  private int index;

  private TunableNumber kP = new TunableNumber("snapToGrid/multiplier", 0.8);

  private TunableNumber rotationKp = new TunableNumber("snapToGrid/rotationKp", 4.9);
  private double rotationOutput;

  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          rotationKp.get(),
          0,
          0,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  public SnapToGrid(Drivetrain drive) {
    // Use addRequirements,() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(drive.getPose().getRotation().getRadians());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(2);
    rotationController.setGoal(targetPose.getRotation().getRadians());
    targetPose = new Pose2d(10000.0, 10000.0, new Rotation2d(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CUBE) {
        for (int i = 1; i <= 3; i++) {
          if (i == 1) {
            index = 1;
          }
          if (i == 2) {
            index = 4;
          }
          if (i == 3) {
            index = 7;
          }
          if (Math.abs(drive.getPose().getY() - bluePos[index].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = bluePos[index];
          }
        }
      } else {
        for (int i = 1; i <= 6; i++) {
          if (i == 1) {
            index = 0;
          }
          if (i == 2) {
            index = 2;
          }
          if (i == 3) {
            index = 3;
          }
          if (i == 4) {
            index = 5;
          }
          if (i == 5) {
            index = 6;
          }
          if (i == 6) {
            index = 8;
          }
          if (Math.abs(drive.getPose().getY() - bluePos[index].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = bluePos[index];
          }
        }
      }
    } else {
      if (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CUBE) {
        for (int i = 1; i <= 3; i++) {
          if (i == 1) {
            index = 1;
          }
          if (i == 2) {
            index = 4;
          }
          if (i == 3) {
            index = 7;
          }
          if (Math.abs(drive.getPose().getY() - redPos[index].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = redPos[index];
          }
        }
      } else {
        for (int i = 1; i <= 6; i++) {
          if (i == 1) {
            index = 0;
          }
          if (i == 2) {
            index = 2;
          }
          if (i == 3) {
            index = 3;
          }
          if (i == 4) {
            index = 5;
          }
          if (i == 5) {
            index = 6;
          }
          if (i == 6) {
            index = 8;
          }
          if (Math.abs(drive.getPose().getY() - redPos[index].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = redPos[index];
          }
        }
      }
    }

    Logger.getInstance().recordOutput("snapToGrid/targetPos", targetPose);

    if (targetPose.getX() > drive.getPose().getX()) {
      xVel = 1 * kP.get();
    } else {
      xVel = -1 * kP.get();
    }

    if (targetPose.getY() > drive.getPose().getY()) {
      yVel = 1 * kP.get();
    } else {
      yVel = -1 * kP.get();
    }

    if (Math.abs(targetPose.getX() - drive.getPose().getX()) < 0.05) {
      xVel = 0;
    }

    if (Math.abs(targetPose.getY() - drive.getPose().getY()) < 0.05) {
      yVel = 0;
    }

    if (rotationController.getGoal().position != targetPose.getRotation().getRadians()) {
      rotationController.setGoal(targetPose.getRotation().getRadians());
    }
    rotationOutput = rotationController.calculate(drive.getPose().getRotation().getRadians());
    drive.drive(xVel, yVel, rotationOutput);

    Logger.getInstance().recordOutput("ActiveCommands/SnapToGrid", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/SnapToGrid", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
