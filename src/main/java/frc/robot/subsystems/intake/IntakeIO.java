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
    public boolean solenoid = false;
    public boolean intakeRevLimitSwitch = false;
    public boolean intakeFwdLimitSwitch = false;
    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double[] intakeTempCelsius = new double[] {};

    public void toLog(LogTable table) {
      table.put("solenoid state", solenoid);
      table.put("intakeRevLimitSwitch", intakeRevLimitSwitch);
      table.put("intakeFwdLimitSwitch", intakeFwdLimitSwitch);
      table.put("IntakeVelocityRPM", intakeVelocityRPM);
      table.put("IntakeAppliedVolts", intakeAppliedVolts);
      table.put("IntakeCurrentAmps", intakeCurrentAmps);
      table.put("IntakeTempCelsius", intakeTempCelsius);
    }

    public void fromLog(LogTable table) {
      solenoid = table.getBoolean("Extended", solenoid);
      intakeRevLimitSwitch = table.getBoolean("intakeRevLimitSwitch", intakeRevLimitSwitch);
      intakeFwdLimitSwitch = table.getBoolean("intakeFwdLimitSwitch", intakeFwdLimitSwitch);
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

  public void resetSensorHeight(double d);
}
