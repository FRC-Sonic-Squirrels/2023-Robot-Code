package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class IntakeIO2023 implements IntakeIO {
  WPI_TalonFX motor = new WPI_TalonFX(Constants.CanId.CANID6_INTAKE_TALON);

  public IntakeIO2023() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 80, 0.2));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // for simplicity's sake just use ticks
    inputs.intakeVelocityRPM = motor.getSelectedSensorVelocity(); // * 10 * 60 / 2048;
    inputs.intakeAppliedVolts = motor.getMotorOutputVoltage();
    inputs.intakeCurrentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.intakeTempCelsius = new double[] {motor.getTemperature()};

    inputs.intakeStatorCurrent = motor.getStatorCurrent();

    // Logger.getInstance().recordOutput("Intake/filteredStatorCurrent", motor.getcurrent);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
