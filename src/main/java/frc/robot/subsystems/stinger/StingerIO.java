// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface StingerIO {

  // Contains all the input data received from hardware
  public static class StingerIOInputs implements LoggableInputs {

    public double StingerExtensionInches = 0.0;
    public double StingerTargetExtensionInches = 0.0;
    public double StingerVelocityInchesPerSecond = 0.0;
    public double StingerVelocityRPM = 0.0;
    public double StingerAppliedVolts = 0.0;
    public double[] StingerCurrentAmps = new double[] {};
    public double[] StingerTempCelsius = new double[] {};

    // is the stinger hitting the lower limit switch?
    public boolean StingerAtRetractedLimit = false;

    // is the stinger hitting the upper limit switch?
    public boolean StingerAtExtendedLimit = false;

    public void toLog(LogTable table) {
      table.put("StingerExtensionInches", StingerExtensionInches);
      table.put("StingerTargetExtensionInches", StingerTargetExtensionInches);
      table.put("StingerAtRetractedLimit", StingerAtRetractedLimit);
      table.put("StingerVelocityInchesPerSecond", StingerVelocityInchesPerSecond);
      table.put("StingerVelocityRPM", StingerVelocityRPM);
      table.put("StingerAppliedVolts", StingerAppliedVolts);
      table.put("StingerCurrentAmps", StingerCurrentAmps);
      table.put("StingerTempCelsius", StingerTempCelsius);
    }

    public void fromLog(LogTable table) {
      StingerAtRetractedLimit =
          table.getBoolean("StingerAtRetractedLimit", StingerAtRetractedLimit);
      StingerAtExtendedLimit = table.getBoolean("StingerAtExtendedLimit", StingerAtExtendedLimit);
      StingerVelocityInchesPerSecond = table.getDouble("StingerVelocityRPM", StingerVelocityRPM);
      StingerVelocityRPM = table.getDouble("StingerVelocityRPM", StingerVelocityRPM);
      StingerAppliedVolts = table.getDouble("StingerAppliedVolts", StingerAppliedVolts);
      StingerCurrentAmps = table.getDoubleArray("StingerCurrentAmps", StingerCurrentAmps);
      StingerTempCelsius = table.getDoubleArray("StingerTempCelsius", StingerTempCelsius);
    }
  }

  // updates the set of loggable inputs
  public default void updateInputs(StingerIOInputs inputs) {}
  ;

  /** Run the Stinger open loop at the specified voltage. */
  public default void setStingerVoltage(double volts) {}

  /** Manually set the output percentage of the stinger */
  public default void setPercent(double percent) {}

  /** set how far out the stinger should be extended, using MotionMagic constraints */
  public default void setExtensionInches(double targetExtensionInches) {}

  /** set MotionMagic constraints, using velocity and acceleration */
  public default void setMotionProfileConstraints(double cruiseVelocity, double acceleration) {}

  public default void setSensorPosition(double position) {}
  ;

  public default void setPIDConstraints(double feedForward, double kP, double kI, double kD) {}
}
