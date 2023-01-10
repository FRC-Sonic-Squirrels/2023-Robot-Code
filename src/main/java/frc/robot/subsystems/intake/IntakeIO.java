// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public boolean extended = false;

    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double[] intakeTempCelsius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Extended", extended);

      table.put("IntakeVelocityRPM", intakeVelocityRPM);
      table.put("IntakeAppliedVolts", intakeAppliedVolts);
      table.put("IntakeCurrentAmps", intakeCurrentAmps);
      table.put("IntakeTempCelsius", intakeTempCelsius);
    }

    public void fromLog(LogTable table) {
      extended = table.getBoolean("Extended", extended);

      intakeVelocityRPM = table.getDouble("IntakeVelocityRPM", intakeVelocityRPM);
      intakeAppliedVolts = table.getDouble("IntakeAppliedVolts", intakeAppliedVolts);
      intakeCurrentAmps = table.getDoubleArray("IntakeCurrentAmps", intakeCurrentAmps);
      intakeTempCelsius = table.getDoubleArray("IntakeTempCelsius", intakeTempCelsius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setIntakeVoltage(double volts) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
