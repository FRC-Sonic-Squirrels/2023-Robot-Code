// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class ElevatorIOInputs implements LoggableInputs {
    public double ElevatorHeightInches = 0.0;
    public double ElevatorVelocityInchesPerSecond = 0.0;
    public boolean ElevatorAtLowerLimit = false;
    public boolean ElevatorAtUpperLimit = false;
    public double ElevatorVelocityRPM = 0.0;
    public double ElevatorAppliedVolts = 0.0;
    public double[] ElevatorCurrentAmps = new double[] {};
    public double[] ElevatorTempCelsius = new double[] {};

    public void toLog(LogTable table) {
      table.put("ElevatorHeightInches", ElevatorHeightInches);
      table.put("ElevatorAtLowerLimit", ElevatorAtLowerLimit);
      table.put("ElevatorVelocityInchesPerSecond", ElevatorVelocityInchesPerSecond);
      table.put("ElevatorVelocityRPM", ElevatorVelocityRPM);
      table.put("ElevatorAppliedVolts", ElevatorAppliedVolts);
      table.put("ElevatorCurrentAmps", ElevatorCurrentAmps);
      table.put("ElevatorTempCelsius", ElevatorTempCelsius);
    }

    public void fromLog(LogTable table) {
      ElevatorAtLowerLimit = table.getBoolean("ElevatorAtLowerLimit", ElevatorAtLowerLimit);
      ElevatorAtUpperLimit = table.getBoolean("ElevatorAtUpperLimit", ElevatorAtUpperLimit);
      ElevatorVelocityInchesPerSecond = table.getDouble("ElevatorVelocityRPM", ElevatorVelocityRPM);
      ElevatorVelocityRPM = table.getDouble("ElevatorVelocityRPM", ElevatorVelocityRPM);
      ElevatorAppliedVolts = table.getDouble("ElevatorAppliedVolts", ElevatorAppliedVolts);
      ElevatorCurrentAmps = table.getDoubleArray("ElevatorCurrentAmps", ElevatorCurrentAmps);
      ElevatorTempCelsius = table.getDoubleArray("ElevatorTempCelsius", ElevatorTempCelsius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the Elevator open loop at the specified voltage. */
  public default void setElevatorVoltage(double volts) {}

  public default void setPercent(double percent) {}

  public default void setHeight(double targetHeightInches) {}

  public default double getHeightInches() {
    // TODO: estimate height for simulation
    return 0.0;
  }

  public default double getHeightTicks() {
    return 0.0;
  }

  public default void zeroHeight() {}

  public default void stop() {}

  public default void brakeOff(){}

  public default void brakeOn(){}

  public default boolean atLowerLimit() {
    return false;
  }

  public default void setMotionMagicSetPoint(double heightInches) {}

  public default boolean isAtHeight(double heightInches) {
    return false;
  }

  public default boolean isAtHeight() {
    return false;
  }

  public default void hold() {}

  public default void setMotionMagicConstraints(double cruiseVelocity, double desiredTimeToSpeed){}

  public default double heightToTicks(double heightInches) {
    return 0.0;
  }

  public default double ticksToHeight(double ticks) {
    return 0.0;
  }
}
