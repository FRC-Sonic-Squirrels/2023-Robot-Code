// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.limelight.Limelight;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class OrbitCube extends CommandBase {
  Limelight limelight;
  Drivetrain drive;

  private TunableNumber rotationKp = new TunableNumber("OrbitCube/rotationKp", 4.9);

  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          rotationKp.get(),
          0,
          0,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;

  private double xVel;
  private double yVel;

  /** Creates a new OrbitCube. */
  public OrbitCube(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Limelight limelight,
      Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
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
    rotationController.setGoal(limelight.getTargetYaw().getRadians() + Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    xVel =
        (-modifyAxis(translationYSupplier.getAsDouble())
                    // * drive.elevatorAndStingerOutTranslationMultiplier.get() TODO: decide if this
                    // is
                    // necessary
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND)
                * Math.cos(limelight.getTargetYaw().getRadians())
            + (-modifyAxis(translationXSupplier.getAsDouble())
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND)
                * Math.cos(limelight.getTargetYaw().getRadians() + Math.PI / 2);

    yVel =
        (-modifyAxis(translationYSupplier.getAsDouble())
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND)
                * Math.sin(limelight.getTargetYaw().getRadians())
            + (-modifyAxis(translationXSupplier.getAsDouble())
                    * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND)
                * Math.sin(limelight.getTargetYaw().getRadians() + Math.PI / 2);

    drive.drive(
        xVel,
        yVel,
        rotationController.calculate(
            drive.getPose().getRotation().getRadians(),
            limelight.getTargetYaw().getRadians() + Math.PI));
    Logger.getInstance().recordOutput("ActiveCommands/OrbitCube", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
    Logger.getInstance().recordOutput("ActiveCommands/OrbitCube", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
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
}
