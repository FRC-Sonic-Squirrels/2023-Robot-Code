package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class IntakeIO2023 implements IntakeIO {
  // TODO make constant can ID
  WPI_TalonFX motor = new WPI_TalonFX(-1);

  public IntakeIO2023() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO check formula
    inputs.intakeVelocityRPM =
        motor.getSensorCollection().getIntegratedSensorVelocity() * 10 * 60 / 2048;

    inputs.intakeAppliedVolts = motor.getMotorOutputVoltage();
    inputs.intakeCurrentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.intakeTempCelsius = new double[] {motor.getTemperature()};
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
