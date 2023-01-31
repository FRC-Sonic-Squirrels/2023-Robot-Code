// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.stinger.StingerIO.StingerIOInputs;

public class Stinger extends SubsystemBase {

  private final StingerIO io;
  private final StingerIOInputs inputs = new StingerIOInputs();
  private double MAX_VOLTAGE = 10.0;
  public static double toleranceInches = 0.05;

  private final TunableNumber kPtunable =
      new TunableNumber("Stinger/kP", 0.48);
  private final TunableNumber feedForwardTunable =
      new TunableNumber("Stinger/FeedForward", 0.054);
  private final TunableNumber kItunable =
      new TunableNumber("Stinger/kI", 0.0);
  private final TunableNumber kDtunable =
      new TunableNumber("Stinger/kD", 0.0);

  /** Creates a new Stinger. */
  public Stinger(StingerIO io) {
    this.io = io;

    io.setSensorPosition(0.0);
    io.setPIDConstraints(0.054, 0.48, 0.0, 0.0);
    //io.setMotionMagicConstraints( FIXME:find velocity and acceleration of stinger );

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Stinger", inputs);

    // update TunableNumbers and change PID accordingly
    if (feedForwardTunable.hasChanged() || kPtunable.hasChanged() || kItunable.hasChanged() || kDtunable.hasChanged()) {
      io.setPIDConstraints(feedForwardTunable.get(), kPtunable.get(), kItunable.get(), kPtunable.get());
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

  public void setLengthInches(double targetLengthInches) {
    io.setExtensionInches(targetLengthInches);
  }

  public double getLengthInches() {
    return inputs.StingerExtensionInches;
  }

  /**
   * @return true if withing tolerance of target length
   */
  public boolean isAtLength(double LengthInches) {
    return (Math.abs(LengthInches - inputs.StingerExtensionInches) < toleranceInches);
  }

  /**
   * isAtLength() check if the Stinger is at the target length.
   *
   * @return true if the Stinger is at the length setpoint
   */
  public boolean isAtLength() {
    return isAtLength(inputs.StingerExtensionInches);
  }

  /** atLowerLimit() returns true if the extended (lower) limit switch is triggered. */
  public boolean atExtendedLimit() {
    return (inputs.StingerAtExtendedLimit);
  }
  public boolean atRetractedLimit() {
    return inputs.StingerAtRetractedLimit;
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

}
