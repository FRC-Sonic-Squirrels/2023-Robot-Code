// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SnapToGrid extends CommandBase {
  /** Creates a new snapToGrid. */
  private final Drivetrain drive;

  private static final Pose2d[] bluePos = {
    new Pose2d(1.93, 0.51, new Rotation2d(Math.toRadians(180))), // CONE /\   WALL SIDE
    new Pose2d(1.93, 1.08, new Rotation2d(Math.toRadians(180))), // CUBE []
    new Pose2d(1.93, 1.62, new Rotation2d(Math.toRadians(180))), // CONE /\
    new Pose2d(1.93, 2.18, new Rotation2d(Math.toRadians(180))), // CONE /\
    new Pose2d(1.93, 2.74, new Rotation2d(Math.toRadians(180))), // CUBE []
    new Pose2d(1.93, 3.31, new Rotation2d(Math.toRadians(180))), // CONE /\
    new Pose2d(1.93, 3.86, new Rotation2d(Math.toRadians(180))), // CONE /\
    new Pose2d(1.93, 4.42, new Rotation2d(Math.toRadians(180))), // CUBE []
    new Pose2d(1.93, 4.98, new Rotation2d(Math.toRadians(180))) // CONE /\   HUMAN PLAYER SIDE
  };

  private static final Pose2d[] redPos = {
    new Pose2d(14.64, 0.51, new Rotation2d(Math.toRadians(0))), // CONE /\   WALL SIDE
    new Pose2d(14.64, 1.08, new Rotation2d(Math.toRadians(0))), // CUBE []
    new Pose2d(14.64, 1.62, new Rotation2d(Math.toRadians(0))), // CONE /\
    new Pose2d(14.64, 2.18, new Rotation2d(Math.toRadians(0))), // CONE /\
    new Pose2d(14.64, 2.74, new Rotation2d(Math.toRadians(0))), // CUBE []
    new Pose2d(14.64, 3.31, new Rotation2d(Math.toRadians(0))), // CONE /\
    new Pose2d(14.64, 3.86, new Rotation2d(Math.toRadians(0))), // CONE /\
    new Pose2d(14.64, 4.42, new Rotation2d(Math.toRadians(0))), // CUBE []
    new Pose2d(14.64, 4.98, new Rotation2d(Math.toRadians(0))) // CONE /\   HUMAN PLAYER SIDE
  };

  private static final int[] cubeIndex = {1, 4, 7};
  private static final int[] coneIndex = {0, 2, 3, 5, 6, 8};

  // private double xVel = 0;
  // private double yVel = 0;

  private Pose2d targetPose = new Pose2d(1000.0, 1000.0, new Rotation2d(0.0));

  private TunableNumber xKp = new TunableNumber("snapToGrid/xKp", 4.0);
  private TunableNumber yKp = new TunableNumber("snapToGrid/yKp", 6.0);

  private TunableNumber xKi = new TunableNumber("snapToGrid/xKi", 0.5);
  private TunableNumber yKi = new TunableNumber("snapToGrid/yKi", 0.5);

  private TunableNumber xKd = new TunableNumber("snapToGrid/xKi", 0);
  private TunableNumber yKd = new TunableNumber("snapToGrid/yKi", 0);

  private TunableNumber rotationKp = new TunableNumber("snapToGrid/rotationKp", 4.9);
  // private double rotationOutput;

  private Trajectory trajectory;

  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          rotationKp.get(),
          0,
          0,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  private PIDController xController = new PIDController(xKp.get(), xKi.get(), xKd.get());
  private PIDController yController = new PIDController(yKp.get(), yKi.get(), yKd.get());

  private HolonomicDriveController driveController =
      new HolonomicDriveController(xController, yController, rotationController);

  private static TunableNumber midPointOffset =
      new TunableNumber("snapToGrid/midPointOffsetMeters", 0.3);
  private static double actualMidPointOffset;

  public static Timer runTime = new Timer();

  // private ProfiledPIDController xController =
  //     new ProfiledPIDController(
  //         xKp.get(),
  //         xKi.get(),
  //         0,
  //         new TrapezoidProfile.Constraints(
  //             DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
  //             DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * 0.9));

  // private ProfiledPIDController yController =
  //     new ProfiledPIDController(
  //         yKp.get(),
  //         yKi.get(),
  //         0,
  //         new TrapezoidProfile.Constraints(
  //             DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
  //             DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * 0.9));

  public SnapToGrid(Drivetrain drive) {
    // Use addRequirements,() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runTime.reset();
    targetPose = new Pose2d(10000.0, 10000.0, new Rotation2d(0));

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
      actualMidPointOffset = midPointOffset.get();
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
      actualMidPointOffset = -midPointOffset.get();
    }

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            drive.getPose(),
            List.of(new Translation2d(targetPose.getX() + actualMidPointOffset, targetPose.getY())),
            targetPose,
            new TrajectoryConfig(
                DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    rotationController.reset(drive.getPose().getRotation().getRadians());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(2);
    rotationController.setGoal(targetPose.getRotation().getRadians());
    xController.reset();
    xController.setTolerance(0.01);
    yController.reset();
    yController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runTime.start();
    Logger.getInstance().recordOutput("snapToGrid/targetPos", targetPose);
    Logger.getInstance().recordOutput("snapToGrid/trajectory", trajectory);

    // xVel = (targetPose.getX() - drive.getPose().getX()) * xKp.get();
    // yVel = (targetPose.getY() - drive.getPose().getY()) * yKp.get();

    // if (rotationController.getGoal().position != targetPose.getRotation().getRadians()) {
    //   rotationController.setGoal(targetPose.getRotation().getRadians());
    // }
    // if (xController.getGoal().position != targetPose.getX()) {
    //   xController.setGoal(targetPose.getX());
    // }
    // if (yController.getGoal().position != targetPose.getY()) {
    //   yController.setGoal(targetPose.getY());
    // }

    // rotationOutput = rotationController.calculate(drive.getPose().getRotation().getRadians());
    // xVel = xController.calculate(drive.getPose().getX(), targetPose.getX());
    // yVel = yController.calculate(drive.getPose().getY(), targetPose.getY());

    // Logger.getInstance().recordOutput("snapToGrid/xVel", xVel);
    // Logger.getInstance().recordOutput("snapToGrid/yVel", yVel);
    // Logger.getInstance().recordOutput("snapToGrid/rotationalOutput", rotationOutput);
    drive.drive(
        driveController.calculate(
            drive.getPose(), trajectory.sample(runTime.get()), targetPose.getRotation()));

    Logger.getInstance().recordOutput("ActiveCommands/SnapToGrid", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runTime.stop();
    drive.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/SnapToGrid", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
