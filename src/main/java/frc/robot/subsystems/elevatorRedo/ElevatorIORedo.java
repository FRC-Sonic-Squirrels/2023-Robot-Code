// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevatorRedo;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIORedo {
  
  // Contains all necessary input data to replay the robot's run
  public static class ElevatorIOInputsRedo implements LoggableInputs {
    public double elevatorHeightInches = 0.0;
    public double elevatorVelocityRPM = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTempCelsius = new double[] {};

    public void toLog(LogTable table) {
      table.put("elevatorHeightInches", elevatorHeightInches);
      table.put("elevatorVelocityRPM", elevatorVelocityRPM);
      table.put("elevatorAppliedVolts", elevatorAppliedVolts);
      table.put("elevatorCurrentAmps", elevatorCurrentAmps);
      table.put("elevatorTempCelsius", elevatorTempCelsius);
    }

    public void fromLog(LogTable table) {
      elevatorHeightInches = table.getDouble("elevatorHeightInches", elevatorHeightInches);
      elevatorVelocityRPM = table.getDouble("elevatorVelocityRPM", elevatorVelocityRPM);
      elevatorAppliedVolts = table.getDouble("elevatorAppliedVolts", elevatorAppliedVolts);
      elevatorCurrentAmps = table.getDoubleArray("elevatorCurrentAmps", elevatorCurrentAmps);
      elevatorTempCelsius = table.getDoubleArray("elevatorTempCelsius", elevatorTempCelsius);
    }
  }

  // Updates the set of loggable inputs
  public default void updateInputs(ElevatorIOInputsRedo inputs) {}

  // Runs the Elevator open loop at a specified voltage
  public default void setElevatorVoltage(double volts) {}

  public default void setMotorPercent(double percent) {}

  public default void setElevatorHeight(double height) {}

}
