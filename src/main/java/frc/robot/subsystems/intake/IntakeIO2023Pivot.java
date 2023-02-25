package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeIO2023Pivot implements IntakeIO {
  // TODO make constant can ID
  WPI_TalonFX motor = new WPI_TalonFX(-1);

  // TODO check channel and pdh type
  Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, -1);

  public IntakeIO2023Pivot() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.solenoid = solenoid.get();

    // TODO check formula
    inputs.intakeVelocityRPM = motor.getSelectedSensorVelocity() * 10 * 60 / 2048;

    inputs.intakeAppliedVolts = motor.getMotorOutputVoltage();
    inputs.intakeCurrentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.intakeTempCelsius = new double[] {motor.getTemperature()};
  }

  @Override
  public void setExtended(boolean extended) {
    solenoid.set(extended);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
