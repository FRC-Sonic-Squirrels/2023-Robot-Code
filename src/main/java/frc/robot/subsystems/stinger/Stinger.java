// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.stinger.StingerIO.StingerIOInputs;
import org.littletonrobotics.junction.Logger;

public class Stinger extends SubsystemBase {

  private final StingerIO io;
  private final StingerIOInputs inputs = new StingerIOInputs();
  // TODO: see if we can change max voltage to 12
  private double MAX_VOLTAGE = 10.0;

  public static double toleranceInches = 0.1;
  private boolean zeroed = false;

  private final TunableNumber feedForwardTunable =
      new TunableNumber("Stinger/FeedForward", Constants.Stinger.STINGER_FEEDFORWARD);
  private final TunableNumber kPtunable =
      new TunableNumber("Stinger/kP", Constants.Stinger.STINGER_KP);
  private final TunableNumber kItunable =
      new TunableNumber("Stinger/kI", Constants.Stinger.STINGER_KI);
  private final TunableNumber kDtunable =
      new TunableNumber("Stinger/kD", Constants.Stinger.STINGER_KD);

  private final TunableNumber velocityInchesSecond =
      new TunableNumber(
          "Stinger/velocity inches per sec", Constants.Stinger.VELOCITY_INCHES_PER_SECOND);
  private final TunableNumber desiredTime = new TunableNumber("Stinger/desired time", 0.1);

  /** Creates a new Stinger. */
  public Stinger(StingerIO io) {
    this.io = io;

    io.setSensorPosition(0.0);
    io.setPIDConstraints(
        feedForwardTunable.get(), kPtunable.get(), kItunable.get(), kDtunable.get());
    setMotionProfileConstraintsTime(velocityInchesSecond.get(), desiredTime.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Stinger", inputs);

    // update TunableNumbers and change PID accordingly
    if (feedForwardTunable.hasChanged()
        || kPtunable.hasChanged()
        || kItunable.hasChanged()
        || kDtunable.hasChanged()) {
      io.setPIDConstraints(
          feedForwardTunable.get(), kPtunable.get(), kItunable.get(), kDtunable.get());
    }
    if (velocityInchesSecond.hasChanged() || desiredTime.hasChanged()) {
      setMotionProfileConstraintsTime(velocityInchesSecond.get(), desiredTime.get());
    }

    // limit switch
    if (inputs.StingerAtRetractedLimit) {
      if (!zeroed) {
        // only zero height once per time hitting limit switch
        io.setSensorPosition(0.0);
        zeroed = true;
      }
    } else {
      // not currently on limit switch, zero again next time we hit limit switch
      zeroed = false;
    }
  }

  /** Run the Stinger at the specified voltage */
  public void runStingerVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    io.setStingerVoltage(voltage * MAX_VOLTAGE);
  }

  public void stop() {
    runStingerVoltage(0.0);
  }

  public void setExtensionInches(double extensionInches) {
    double targetInches =
        MathUtil.clamp(extensionInches, 0.0, Constants.Stinger.MAX_EXTENSION_INCHES);

    io.setExtensionInches(targetInches);
  }

  public double getExtensionInches() {
    return inputs.StingerExtensionInches;
  }

  /**
   * @return true if within tolerance of a target extension
   */
  public boolean isAtExtension(double extensionInches) {
    return (Math.abs(extensionInches - inputs.StingerExtensionInches) < toleranceInches);
  }

  /**
   * isAtExtension() check if the Stinger is at the target extension.
   *
   * @return true if the Stinger is at the extension setpoint
   */
  public boolean isAtExtension() {
    return isAtExtension(inputs.StingerTargetExtensionInches);
  }

  /** atRetractedLimit() returns true if the retracted (lower) limit switch is triggered. */
  public boolean atRetractedLimit() {
    return (inputs.StingerAtRetractedLimit);
  }

  public boolean atExtendedLimit() {
    return inputs.StingerAtExtendedLimit;
  }

  public double getSpeedRPM() {
    return inputs.StingerVelocityRPM;
  }

  public double getSpeedInchesPerSecond() {
    return inputs.StingerVelocityInchesPerSecond;
  }

  public void zeroExtension() {
    io.setSensorPosition(0.0);
  }

  public void setMotionProfileConstraintsTime(double velocity, double time) {
    double acceleration = velocity / time;
    io.setMotionProfileConstraints(velocity, acceleration);
  }

  public void setPercentOutput(double percent) {
    io.setPercent(percent);
  }
}
