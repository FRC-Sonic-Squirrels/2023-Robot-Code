// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.limelight.Limelight;
import org.littletonrobotics.junction.Logger;

public class DriveToCube extends CommandBase {
  /** Creates a new DriveToCube. */
  private final Limelight limelight;

  private final Drivetrain drive;

  private TunableNumber rotationKp = new TunableNumber("DriveToCube/rotationKp", 4.9);

  private TunableNumber xKp = new TunableNumber("DriveToCube/xKp", 3.2);
  private TunableNumber yKp = new TunableNumber("DriveToCube/yKp", 3.2);

  private TunableNumber xKi = new TunableNumber("DriveToCube/xKi", 0.0);
  private TunableNumber yKi = new TunableNumber("DriveToCube/yKi", 0.0);

  private TunableNumber xKd = new TunableNumber("DriveToCube/xKd", 0);
  private TunableNumber yKd = new TunableNumber("DriveToCube/yKd", 0);

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

  private double rotationalErrorDegrees;
  private double xVel;
  private double yVel;
  private double rotVel;
  private TunableNumber allowedRotationalErrorDegrees =
      new TunableNumber("DriveToCube/allowedRotationalErrorDegrees", 20);

  private TunableNumber advancedMode =
      new TunableNumber("DriveToCube/advancedMode/doAdvancedMotion", 0);

  public DriveToCube(Limelight limelight, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(drive.getPose().getRotation().getRadians());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(2);
    rotationController.setGoal(limelight.getTargetYaw().getRadians());

    xController.reset();
    xController.setTolerance(0.01);

    yController.reset();
    yController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotationalErrorDegrees =
        Math.abs(
            limelight.getTargetYaw().getDegrees()
                - 180.0
                - drive.getPose().getRotation().getDegrees());
    Logger.getInstance().recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

    if (advancedMode.get() == 0) {
      xVel =
          rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
              ? xController.calculate(drive.getPose().getX(), limelight.getCubePoseMeters().getX())
              : 0.0;
      yVel =
          rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
              ? yController.calculate(drive.getPose().getY(), limelight.getCubePoseMeters().getY())
              : 0.0;
    } else {
      xVel =
          xController.calculate(drive.getPose().getX(), limelight.getCubePoseMeters().getX())
              / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegrees.get(), 1.0);
      yVel =
          yController.calculate(drive.getPose().getY(), limelight.getCubePoseMeters().getY())
              / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegrees.get(), 1.0);
    }

    rotVel =
        rotationController.calculate(
            drive.getPose().getRotation().getRadians(),
            limelight.getTargetYaw().getRadians() + Math.PI);

    drive.drive(xVel, yVel, rotVel);

    Logger.getInstance().recordOutput("DriveToCube/rotationalErrorDegrees", rotationalErrorDegrees);
    Logger.getInstance().recordOutput("DriveToCube/xVel", xVel);
    Logger.getInstance().recordOutput("DriveToCube/yVel", yVel);
    Logger.getInstance().recordOutput("DriveToCube/rotVel", rotVel);

    Logger.getInstance().recordOutput("ActiveCommands/DriveToCube", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/DriveToCube", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
