package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.lib.team2930.lib.util.MotorUtils;
import frc.robot.Constants.CANIVOR_canId;

public class ElevatorReal2023 implements ElevatorIO {

  private WPI_TalonFX lead_talon =
      new WPI_TalonFX(CANIVOR_canId.CANID9_ELEVATOR_LEAD_TALON, CANIVOR_canId.name);
  private WPI_TalonFX follow_talon =
      new WPI_TalonFX(CANIVOR_canId.CANID10_ELEVATOR_FOLLOW_TALON, CANIVOR_canId.name);
  private static final double gearRatio = 0.072; // (12 * 18) / (50 * 60)
  private static final double pulleyDiameterInches = 1.75;
  private static final double pulleyCircumference = Math.PI * pulleyDiameterInches;
  // we multiply ticks2inches by 2, because it's a 2 stage cascading elevator
  private static final double ticks2inches = 2.0 * gearRatio * pulleyCircumference / 2048;
  private static final double ticks2rotations = 1 / 2048f;
  private double targetHeightInches;
  private double maxHeightInches = 0.0;

  public ElevatorReal2023() {
    lead_talon.configFactoryDefault();
    follow_talon.configFactoryDefault();

    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // TODO: PIDF values are set later with setPIDConstraints() from Elevator class?
    leadConfig.slot0.integralZone = 0.0;
    leadConfig.slot0.closedLoopPeakOutput = 1.0;

    leadConfig.slot0.allowableClosedloopError = Elevator.toleranceInches / ticks2inches;

    // set config
    lead_talon.configAllSettings(leadConfig);

    // configure integrated sensor as selected sensor
    lead_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // use pid from slot 0 for motion magic
    lead_talon.selectProfileSlot(0, 0);

    lead_talon.setNeutralMode(NeutralMode.Brake);
    follow_talon.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    // TODO: remember to configure
    lead_talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    // No physical forward limit switch
    // lead_talon.configForwardLimitSwitchSource(
    //     LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);

    follow_talon.follow(lead_talon);
    // TODO: check to see whether inverted or not
    lead_talon.setInverted(false);
    follow_talon.setInverted(false);

    // JVN calculator suggests 8Amp max
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 10, 25, 0.1);
    lead_talon.configSupplyCurrentLimit(currentLimit);
    follow_talon.configSupplyCurrentLimit(currentLimit);

    // NOTE: only effects manual control
    lead_talon.configOpenloopRamp(0.1);

    // TODO: determine whether we actually want to make this slow
    MotorUtils.setCtreStatusSlow(follow_talon);

    // TODO make sure this doesn't slow down CAN to anything important
    follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 41);
    follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 201);

    // TODO: make sure this works
    lead_talon.configForwardSoftLimitThreshold(inchesToTicks(maxHeightInches));
    lead_talon.configForwardSoftLimitEnable(true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (lead_talon.isRevLimitSwitchClosed() == 1) {
      inputs.ElevatorAtLowerLimit = true;
    } else {
      inputs.ElevatorAtLowerLimit = false;
    }

    inputs.ElevatorTargetHeightInches = targetHeightInches;
    inputs.ElevatorHeightInches = ticksToInches(lead_talon.getSelectedSensorPosition());
    inputs.ElevatorAtUpperLimit = (inputs.ElevatorHeightInches >= maxHeightInches);

    // no physical upper limit switch
    // if (lead_talon.isFwdLimitSwitchClosed() == 1) {
    //   inputs.ElevatorAtUpperLimit = true;
    // } else {
    //   inputs.ElevatorAtUpperLimit = false;
    // }

    inputs.ElevatorAppliedVolts = lead_talon.getMotorOutputVoltage();
    inputs.ElevatorCurrentAmps = new double[] {lead_talon.getSupplyCurrent()};
    inputs.ElevatorTempCelsius = new double[] {lead_talon.getTemperature()};
    // NOTE: don't use talon.getSensorCollection() see:
    // https://www.chiefdelphi.com/t/swerve-odometry-problems/392680/5
    double sensorVelocity = lead_talon.getSelectedSensorVelocity();
    inputs.ElevatorVelocityInchesPerSecond = ticksToInchesPerSecond(sensorVelocity);
    inputs.ElevatorVelocityRPM = sensorVelocity * 10.0 * ticks2rotations / 60.0;
  }

  /**
   * setMotionProfileConstraints - set the max velocity and acceleration of motion profile.
   *
   * <p>velocity is inches/sec acceleration is inches/s^2
   */
  @Override
  public void setMotionProfileConstraints(
      double cruiseVelocityInchesPerSecond, double accelerationInchesPerSecondSquared) {

    double velocityTicksPer100ms = inchesToTicksPer100ms(cruiseVelocityInchesPerSecond);
    double accelTicksPer100msPerSecond = inchesToTicksPer100ms(accelerationInchesPerSecondSquared);

    lead_talon.configMotionCruiseVelocity(velocityTicksPer100ms);
    lead_talon.configMotionAcceleration(accelTicksPer100msPerSecond);
  }

  public void setMaxHeightInches(double inches) {
    maxHeightInches = inches;
  }

  private double inchesToTicks(double inches) {
    return inches / ticks2inches;
  }

  private double ticksToInches(double ticks) {
    return ticks * ticks2inches;
  }

  /**
   * Convert form inches/s to CTRE ticks / 100ms
   *
   * @param inchesPerSecond
   * @return ticksPer100ms
   */
  private double inchesToTicksPer100ms(double inchesPerSecond) {
    return inchesToTicks(inchesPerSecond) / 10.0;
  }

  /**
   * convert from CTRE ticks / 100ms to Inches/s
   *
   * @param ticksPer100ms
   * @return inchesPerSecond
   */
  private double ticksToInchesPerSecond(double ticksPer100ms) {
    return ticksToInches(ticksPer100ms) * 10;
  }

  @Override
  public void resetSensorHeight(double heightInches) {
    lead_talon.setSelectedSensorPosition(inchesToTicks(heightInches));
    // lead_talon.getSensorCollection().setIntegratedSensorPosition(inchesToTicks(heightInches), 0);
  }

  @Override
  public void setPercent(double percent) {
    lead_talon.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setPIDConstraints(double kF, double kP, double kI, double kD) {
    lead_talon.config_kF(0, kF);
    lead_talon.config_kP(0, kP);
    lead_talon.config_kI(0, kI);
    lead_talon.config_kD(0, kD);
  }

  @Override
  public void setHeightInches(double targetHeightInches) {
    lead_talon.set(TalonFXControlMode.Position, inchesToTicks(targetHeightInches));
  }

  @Override
  public void setElevatorVoltage(double volts) {
    lead_talon.setVoltage(volts);
  }
}
