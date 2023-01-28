// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.stinger.StingerIO.StingerIOInputs;

public class Stinger extends SubsystemBase {

  private final StingerIO io;
  private final StingerIOInputs inputs = new StingerIOInputs();
  private double MAX_VOLTAGE = 10.0;
  public static double toleranceInches = 0.05;

  /** Creates a new Stinger. */
  public Stinger(StingerIO io) {
    this.io = io;
    io.zeroLength();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Stinger", inputs);
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
    io.setLengthInches(targetLengthInches);
  }

  public double getLengthInches() {
    return inputs.StingerLengthInches;
  }

  /**
   * @return true if withing tolerance of target length
   */
  public boolean isAtLength(double LengthInches) {
    return (Math.abs(LengthInches - inputs.StingerLengthInches) < toleranceInches);
  }

  /**
   * isAtLength() check if the Stinger is at the target length.
   *
   * @return true if the Stinger is at the length setpoint
   */
  public boolean isAtLength() {
    return isAtLength(inputs.StingerLengthInches);
  }

  /** atLowerLimit() returns true if the lower limit switch is triggered. */
  public boolean atLowerLimit() {
    return (inputs.StingerAtLowerLimit);
  }

  // TODO: implement methods to get upper and lower limit switch status
  // TODO: implement methods to get Stinger motor RPM, velocity in/s
}
