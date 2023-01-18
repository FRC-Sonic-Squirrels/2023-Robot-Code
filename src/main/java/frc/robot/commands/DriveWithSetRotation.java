// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;


/*
 * This class implements field centric swerve drive, with fixed rotational control
 * The robot defaults to zero degree rotation, but the XBox POV buttons change the angle
 */
public class DriveWithSetRotation extends CommandBase {

  private final Drivetrain m_drivetrain;

  // input suppliers from joysticks
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationPOVSupplier;

  // rotation to be held by the robot while driving, in radians
  private double m_setRotationRadians;

  // PID controller to maintain fixed rotation, with P being a TunableNumber
  private ProfiledPIDController rotationController = new ProfiledPIDController(4.9, 0, 0,
    new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  /** Creates a new DriveWithSetRotation. */
  public DriveWithSetRotation(Drivetrain drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier pov, double rotationRadians) {

    m_drivetrain = drive;

    m_translationXSupplier = x;
    m_translationYSupplier = y;
    m_rotationPOVSupplier = pov;

    m_setRotationRadians = rotationRadians;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(rotationRadians);

    SmartDashboard.putNumber("Target Angle Radians", m_setRotationRadians);
    SmartDashboard.putNumber("Current Robot Angle Radians", m_drivetrain.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Current Robot Angle Velocity Radians", rotationRadians);
    SmartDashboard.putNumber("Rotation Output", 0.0);

    SmartDashboard.putNumber("Angle Error Radians", rotationController.getPositionError());
    SmartDashboard.putNumber("Angle Velocity Error Radians", rotationController.getVelocityError());

    SmartDashboard.putBoolean("Angle at Target", rotationController.atGoal());

    SmartDashboard.putNumber("X Position", m_translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Y Position", m_translationYSupplier.getAsDouble());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resets robot position to its current measured position
    rotationController.reset(m_drivetrain.getPose().getRotation().getRadians());
    rotationController.setGoal(m_setRotationRadians);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double pov = m_rotationPOVSupplier.getAsDouble();

    // if pov was < 0 that would mean theres no input
    if (pov >= 0) {

      // angle correction
      if (pov > 180) {
        pov = 360 - pov;
      }
      else {
        pov = -pov;
      }

      if (Math.toRadians(pov) != m_setRotationRadians) {

        // only reset PID is target angle changes
        SmartDashboard.putNumber("TargetAngle", pov);
        m_setRotationRadians = Math.toRadians(pov);
      }
    }

    double rotationOutput = rotationController.calculate(m_drivetrain.getPose().getRotation().getRadians(), m_setRotationRadians);

    if (Math.abs(rotationOutput) < 0.05) {
      rotationOutput = 0;
    }

    //m_drivetrain.drive(rotationOutput, pov, rotationOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
