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
  /** Creates a new IntakeCube. */
  private final Limelight limelight;

  private final Drivetrain drive;

  private TunableNumber rotationKp = new TunableNumber("intakeCube/rotationKp", 4.9);

  private TunableNumber xKp = new TunableNumber("intakeCube/xKp", 3.2);
  private TunableNumber yKp = new TunableNumber("intakeCube/yKp", 3.2);

  private TunableNumber xKi = new TunableNumber("intakeCube/xKi", 0.0);
  private TunableNumber yKi = new TunableNumber("intakeCube/yKi", 0.0);

  private TunableNumber xKd = new TunableNumber("intakeCube/xKd", 0);
  private TunableNumber yKd = new TunableNumber("intakeCube/yKd", 0);

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
  private TunableNumber allowedRotationalErrorDegrees =
      new TunableNumber("intakeCube/allowedRotationalErrorDegrees", 20);

  public DriveToCube(Limelight limelight, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drive = drive;
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

    if (rotationController.getGoal().position != limelight.getTargetYaw().getRadians()
        && limelight.isValidTarget()) {
      rotationController.setGoal(limelight.getTargetYaw().getRadians());
    }

    rotationalErrorDegrees =
        Math.abs(
            drive.getPose().getRotation().getDegrees()
                - limelight.getCubePoseMeters().getRotation().getDegrees());
    Logger.getInstance().recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

    xVel =
        rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
            ? xController.calculate(drive.getPose().getX(), limelight.getCubePoseMeters().getX())
            : 0.0;
    yVel =
        rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
            ? yController.calculate(drive.getPose().getY(), limelight.getCubePoseMeters().getY())
            : 0.0;

    drive.drive(
        xVel, yVel, rotationController.calculate(drive.getPose().getRotation().getRadians()));

    Logger.getInstance().recordOutput("ActiveCommands/IntakeCube", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/IntakeCube", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
