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
    new Pose2d(1.88, 0.55, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 1.10, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 1.66, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 2.21, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 2.76, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 3.33, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 3.88, new Rotation2d(Math.toRadians(180))), // CONE
    new Pose2d(1.88, 4.44, new Rotation2d(Math.toRadians(180))), // CUBE
    new Pose2d(1.88, 5.02, new Rotation2d(Math.toRadians(180)))
  }; // CONE

  private static final Pose2d[] redPos = {
    new Pose2d(14.69, 0.55, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 1.10, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 1.66, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 2.21, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 2.76, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 3.33, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 3.88, new Rotation2d(Math.toRadians(0))), // CONE
    new Pose2d(14.69, 4.44, new Rotation2d(Math.toRadians(0))), // CUBE
    new Pose2d(14.69, 5.02, new Rotation2d(Math.toRadians(0)))
  }; // CONE

  private static final int[] cubeIndex = {1, 4, 7};
  private static final int[] coneIndex = {0, 2, 3, 5, 6, 8};

  private double xVel = 0;
  private double yVel = 0;

  private Pose2d targetPose = new Pose2d(1000.0, 1000.0, new Rotation2d(0.0));

  private TunableNumber xKp = new TunableNumber("snapToGrid/xKp", 0.8);
  private TunableNumber yKp = new TunableNumber("snapToGrid/yKp", 1.2);

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
        for (int i = 0; i <= 2; i++) {
          if (Math.abs(drive.getPose().getY() - bluePos[cubeIndex[i]].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = bluePos[cubeIndex[i]];
          }
        }
      } else {
        for (int i = 0; i <= 5; i++) {
          if (Math.abs(drive.getPose().getY() - bluePos[coneIndex[i]].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = bluePos[coneIndex[i]];
          }
        }
      }
    } else {
      if (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CUBE) {
        for (int i = 0; i <= 2; i++) {
          if (Math.abs(drive.getPose().getY() - redPos[cubeIndex[i]].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = redPos[cubeIndex[i]];
          }
        }
      } else {
        for (int i = 0; i <= 5; i++) {
          if (Math.abs(drive.getPose().getY() - redPos[coneIndex[i]].getY())
              < Math.abs(drive.getPose().getY() - targetPose.getY())) {
            targetPose = redPos[coneIndex[i]];
          }
        }
      }
    }

    Logger.getInstance().recordOutput("snapToGrid/targetPos", targetPose);

    if (targetPose.getX() > drive.getPose().getX()) {
      xVel = 1 * xKp.get();
    } else {
      xVel = -1 * xKp.get();
    }

    if (targetPose.getY() > drive.getPose().getY()) {
      yVel = 1 * yKp.get();
    } else {
      yVel = -1 * yKp.get();
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

    Logger.getInstance().recordOutput("snapToGrid/xVel", xVel);
    Logger.getInstance().recordOutput("snapToGrid/yVel", yVel);
    Logger.getInstance().recordOutput("snapToGrid/rotationalOutput", rotationOutput);
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
