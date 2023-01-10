// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeIOFalcon implements IntakeIO {

  private WPI_TalonFX intakeMotor;

  private final Solenoid solenoid;

  public IntakeIOFalcon() {

    // FIXME: add solenoid channel to constants
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);

    // FIXME: add this id a CAN ID to constants
    intakeMotor = new WPI_TalonFX(18);

    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    // intakeMotor.configVoltageCompSaturation(10.0);
    // intakeMotor.enableVoltageCompensation(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extended = solenoid.get();

    inputs.intakeVelocityRPM =
        intakeMotor.getSensorCollection().getIntegratedSensorVelocity() * 600 / 2048;

    inputs.intakeAppliedVolts = intakeMotor.getMotorOutputVoltage();
    inputs.intakeCurrentAmps = new double[] {intakeMotor.getSupplyCurrent()};
    inputs.intakeTempCelsius = new double[] {intakeMotor.getTemperature()};
  }

  @Override
  public void setIntakeVoltage(double volts) {
    volts = MathUtil.clamp(volts, -10.0, 10.0);
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setExtended(boolean extended) {
    solenoid.set(extended);
  }
}
