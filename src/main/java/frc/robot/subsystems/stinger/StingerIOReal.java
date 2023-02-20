// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.CANIVOR_canId;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class StingerIOReal implements StingerIO {

  // The stinger has similar code to the elevator, but it only has 1 motor and no brake (and it is
  // horizontal)
  private WPI_TalonFX motor =
      new WPI_TalonFX(CANIVOR_canId.CANID5_STINGER_TALON, CANIVOR_canId.name);

  /*
  // motor's native units: ticks/100ms, can be converted to RPM
  // the RPM is applied to the gear ratio,
  // then used to find RPM of pulley
  // pulley RPM -> teeth/rotation -> teeth/inch
  // ticks/second -> motor revolutions/minute -> pulley rpm (account gears) -> teeth/revolution -> teeth/inch
  private final double pulleyTeeth = 20;
  private final double gearRatio = 0.1;
  private final double beltTeethPerInch = 10;
  // private double motorRPM = motor.getSelectedSensorVelocity() * (10/2048) * 60;
  // private final double pulleyRPM = motorRPM * gearRatio;
  // private final double pulleyTeethPerMinute = pulleyRPM * pulleyTeeth;
  // private final double beltInchesPerMinute = pulleyTeethPerMinute / beltTeethPerInch;
  // private final double beltInchesPerSecond = beltInchesPerMinute / 60;
  // private double ticks2VelocityInchesSecond =
  //     (10 * gearRatio * pulleyTeeth) / (2048 * beltTeethPerInch);
  private double ticks2distanceInches =
      (motor.getSelectedSensorPosition() * gearRatio * pulleyTeeth / (2048 * beltTeethPerInch));
  */

  private final double gearRatio = 0.144; // (12 * 30) / (50 * 50)
  private final double pulleyDiameterInches = 1.125;
  private final double pulleyCircumference = pulleyDiameterInches * Math.PI;
  // Multiplied by 2 because of the continuous elevator; double the elevator moves
  private final double cascadeMultiplier = 2.0;
  private double ticks2Inches = gearRatio * pulleyCircumference * cascadeMultiplier / 2048.0;

  private final double maxExtensionInches = Constants.Stinger.MAX_EXTENSION_INCHES;

  public boolean atMaxExtension;

  public double currentExtension;
  public double targetExtensionInches;

  public StingerIOReal() {
    motor.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggest trapezoidal velocity profile constants.

    // do we need this if the command is updating the motion magic constraints?
    // maybe have a safe default?
    // config.motionAcceleration = 30000; // 60941;    //  20521 ticks/100ms     = 11 in/s
    // config.motionCruiseVelocity = 15235; //  20521 ticks/100ms/sec = 11 in/s^2

    config.slot0.allowableClosedloopError = Stinger.toleranceInches / ticks2Inches;

    // set config
    motor.configAllSettings(config);

    // configure integrated sensor as selected sensor
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // use pid from slot 0 for motion magic
    motor.selectProfileSlot(0, 0);

    motor.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    motor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    // TODO: check if motor should be inverted or not
    motor.setInverted(false);

    // JVN calculator predicts 41.2 A per motor under load
    // TODO: Check new JVN prediction
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
    motor.configSupplyCurrentLimit(currentLimit);

    // this only effects manual control
    motor.configOpenloopRamp(0.1);

    setSensorPosition(0.0);

    // TODO: see if we need this soft limit, since we already have a limit switch
    motor.configForwardSoftLimitThreshold(extensionToTicks(maxExtensionInches));
    motor.configForwardSoftLimitEnable(true);
  }

  public void setStingerVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void setPercent(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setExtensionInches(double extensionInches) {
    // if (heightInches < 0.0) {
    //   heightInches = 0.0;
    // }
    if (extensionInches > maxExtensionInches) {
      extensionInches = maxExtensionInches;
    }

    motor.set(ControlMode.MotionMagic, extensionToTicks(extensionInches));
  }

  /**
   * @param acceleration accel in inches per second^2
   * @param cruiseVelocity max velocity in inches per second
   */
  @Override
  public void setMotionProfileConstraints(double cruiseVelocity, double acceleration) {
    // math adapted from howdyBots jvn calculator equation
    // TODO: if speed and acceleration are strange during testing, check to see if these values are
    // correct
    double veloInTicks = cruiseVelocity; // * (12.15 / winchCircumference) * 2048 / 10;
    double accelInTicks = acceleration; // TODO: inches to ticks

    motor.configMotionAcceleration(accelInTicks);
    motor.configMotionCruiseVelocity(veloInTicks);

    // temporary for debugging
    Logger.getInstance().recordOutput("Stinger Constraint Accel INCHES", acceleration);
    Logger.getInstance().recordOutput("Stinger Constraint velo INCHES", cruiseVelocity);
    Logger.getInstance().recordOutput("Stinger Constraint accel TICKS", accelInTicks);
    Logger.getInstance().recordOutput("Stinger Constraint velo TICKS", veloInTicks);
  }

  public void resetSensorPosition(double position) {
    motor.setSelectedSensorPosition(position);
  }

  public void setPIDConstraints(double feedForward, double kP, double kI, double kD) {

    motor.config_kF(0, feedForward);
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    // motor.config_IntegralZone(0, 0);
    // motor.configClosedLoopPeakOutput(0, 1);
  }

  public double extensionToTicks(double extensionInches) {
    return extensionInches / ticks2Inches;
  }

  public double ticksToExtensionInches(double ticks) {
    return ticks * ticks2Inches;
  }

  /**
   * Convert form inches/s to CTRE ticks / 100ms
   *
   * @param inchesPerSecond
   * @return ticksPer100ms
   */
  private double inchesToTicksPer100ms(double inchesPerSecond) {
    return extensionToTicks(inchesPerSecond) / 10.0;
  }

  /**
   * convert from CTRE ticks / 100ms to Inches/s
   *
   * @param ticksPer100ms
   * @return inchesPerSecond
   */
  private double ticksToInchesPerSecond(double ticksPer100ms) {
    return ticksToExtensionInches(ticksPer100ms) * 10;
  }
}
