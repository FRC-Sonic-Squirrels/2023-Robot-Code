// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// code based off RI3D's beam balance command:
// https://github.com/GOFIRST-Robotics/Ri3D-2023/blob/6d79b376bde95481b32f0a98edbf424580653960/src/main/java/frc/robot/commands/BalanceOnBeamCommand.java

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoEngage extends CommandBase {

  private Drivetrain drivetrain;

  private Timer timeEngaged = new Timer();
  private double error;
  private double currentPitch;
  private double drivePower;
  private boolean flip;

  private TunableNumber Kp = new TunableNumber("AutoEngage/kp", 0.05);
  private TunableNumber balancedThresholdDegrees =
      new TunableNumber("AutoEngage/balancedThresholdDegrees", 3);
  // private TunableNumber timeRequiredBalanced =
  //     new TunableNumber("AutoEngage/timeRequiredBalanced", 1);
  private TunableNumber maxPowerPercent = new TunableNumber("AutoEngage/maxPowerPercent", 1);

  private TunableNumber doDrive = new TunableNumber("AutoEngage/EnableCommand", 1);

  private DoubleSupplier y_supplier;
  /** Creates a new AutoEngage. */
  public AutoEngage(Drivetrain drivetrain, Boolean flip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.flip = flip;
    // y_supplier = yAxisSup;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // currentPitch = -1 * -modifyAxis(y_supplier.getAsDouble()) * 45;

    this.currentPitch = drivetrain.getGyroPitch();

    error = currentPitch;
    drivePower = (Kp.get() * error);

    // drivePower = Math.copySign(drivePower, error);

    if (flip) {
      drivePower *= -1;
    }
    // The robot I referenced when making this needed extra power while in reverse.
    //  // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    //  if (drivePower < 0) {
    //    drivePower *= DrivetrainConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    //  }

    // Limit the max power
    if (Math.abs(drivePower)
        > DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * maxPowerPercent.get()) {
      drivePower =
          Math.copySign(
              DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * maxPowerPercent.get(),
              drivePower);
    }

    if (doDrive.get() == 1) {
      drivetrain.drive(drivePower, 0, 0);
    }

    // Debugging Print Statements
    Logger.getInstance().recordOutput("AutoEngage/Current Angle", currentPitch);
    Logger.getInstance().recordOutput("AutoEngage/Error", error);
    Logger.getInstance().recordOutput("AutoEngage/Drive Power", drivePower);
    Logger.getInstance().recordOutput("AutoEngage/timeEngaged", timeEngaged.get());

    // Starts the timer when we are within the specified threshold of being 'flat' (gyroscope pitch
    // of 0 degrees)
    if (Math.abs(error) <= balancedThresholdDegrees.get()) {
      timeEngaged.start();
    } else {
      timeEngaged.reset();
    }

    if (timeEngaged.get() >= 0.5) {
      // drivetrain.enableXstance();
    } else {
      // drivetrain.disableXstance();
    }
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.setXStance();
    // the wheels stay at their last known rotation, even if this command ends the wheels will point
    // x stance until driver gives input. We want to disable x stance so that the driver can regain
    // control in teleop
    drivetrain.disableXstance();
    // drivetrain.enableFieldRelative();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (timeEngaged.get()>timeRequiredBalanced.get());
    return false;
  }
}
