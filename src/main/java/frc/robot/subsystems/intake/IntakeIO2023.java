package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.robot.Constants.CANIVOR_canId;

public class IntakeIO2023 implements IntakeIO {
  // TODO make constant can ID
  WPI_TalonFX motor = new WPI_TalonFX(-1);

  private WPI_TalonFX talonFX = new WPI_TalonFX(CANIVOR_canId.CANID12_INTAKE_TALON, CANIVOR_canId.name);

  public IntakeIO2023() {
    talonFX.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();

    talonFX.configAllSettings(config);

    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configVoltageCompSaturation(10.0);
    motor.enableVoltageCompensation(true);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 17);
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 41);
   
   
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO check formula
    inputs.intakeVelocityRPM = motor.getSelectedSensorVelocity() * 10 * 60 / 2048;

    inputs.intakeAppliedVolts = motor.getMotorOutputVoltage();
    inputs.intakeCurrentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.intakeTempCelsius = new double[] {motor.getTemperature()};
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
