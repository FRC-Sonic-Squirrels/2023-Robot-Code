// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/*
 * This class implements field centric swerve drive, with fixed rotational control
 * The robot defaults to zero degree rotation, but the XBox POV buttons change the angle
 */
public class DriveWithSetRotation extends CommandBase {

  private final Drivetrain m_drivetrain;

  // input suppliers from joysticks
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private double m_rotationPOV;

  // rotation to be held by the robot while driving, in radians
  private double m_setRotationRadians;
  private double DRIVE_ROTATION_KP = 4.9;
  private double TOLERANCE_DEGREES_DEFAULT_VALUE = 5;
  private final TunableNumber rotationKp =
      new TunableNumber("Drive/DriveSetRotation/kP", DRIVE_ROTATION_KP);
  private final TunableNumber toleranceDegrees =
      new TunableNumber("Drive/DriveSetRotation/toleranceDegrees", TOLERANCE_DEGREES_DEFAULT_VALUE);
  // PID controller to maintain fixed rotation, with P being a TunableNumber
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          rotationKp.get(),
          0,
          0,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  /** Creates a new DriveWithSetRotation. */
  public DriveWithSetRotation(Drivetrain drive, DoubleSupplier x, DoubleSupplier y, double pov) {
    m_drivetrain = drive;

    m_translationXSupplier = x;
    m_translationYSupplier = y;
    m_rotationPOV = pov;

    // m_setRotationRadians = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(toleranceDegrees.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resets robot position to its current measured position
    rotationController.reset(m_drivetrain.getPose().getRotation().getRadians());

    m_setRotationRadians = Units.degreesToRadians(m_rotationPOV);

    if (DriverStation.getAlliance() == Alliance.Red) {
      m_setRotationRadians = Units.degreesToRadians(180) + m_setRotationRadians;
    }

    rotationController.setGoal(m_setRotationRadians);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xPercentage = -modifyAxis(m_translationXSupplier.getAsDouble());
    double yPercentage = -modifyAxis(m_translationYSupplier.getAsDouble());

    double xVelocity = xPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double yVelocity = yPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    // if pov was < 0 that would mean theres no input

    double rotationOutput =
        rotationController.calculate(m_drivetrain.getPose().getRotation().getRadians());

    if (Math.abs(rotationOutput) < 0.05) {
      rotationOutput = 0;
    }
    m_drivetrain.drive(xVelocity, yVelocity, rotationOutput);
    // m_drivetrain.drive(rotationOutput, pov, rotationOutput);
    if (rotationKp.hasChanged()) {
      rotationController.setPID(rotationKp.get(), 0, 0);
    }
    if (toleranceDegrees.hasChanged()) {
      rotationController.setTolerance(toleranceDegrees.get());
    }
    Logger.getInstance().recordOutput("ActiveCommands/DriveWithSetRotation", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/DriveWithSetRotation", false);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DrivetrainConstants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
