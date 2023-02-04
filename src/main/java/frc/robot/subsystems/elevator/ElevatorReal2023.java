package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
  // TODO: Put in values of new robot
  private static final double gearRatio = 0.08229;
  private static final double maxExtensionInches = 26;
  private static final double winchDiameter_inches = 1.43;
  private static final double winchCircumfrence = Math.PI * winchDiameter_inches;
  private static final double ticks2inches = gearRatio * winchCircumfrence / 2048;
  private static final double ticks2rotations = 1 / 2048f;
  private double targetHeightInches;

  public ElevatorReal2023() {
    lead_talon.configFactoryDefault();
    follow_talon.configFactoryDefault();

    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggest trapezoidal velocity profile constants.
    // leadConfig.slot0.integralZone = 0.0;
    // leadConfig.slot0.closedLoopPeakOutput = 1.0;

    // TODO: Check these values for 2023 robot
    // leadConfig.motionAcceleration = 30000;
    // leadConfig.motionCruiseVelocity = 15235;

    leadConfig.slot0.allowableClosedloopError = Elevator.toleranceInches / ticks2inches;

    // set config
    lead_talon.configAllSettings(leadConfig);

    // use pid from slot 0 for motion magic
    lead_talon.selectProfileSlot(0, 0);

    lead_talon.setNeutralMode(NeutralMode.Brake);
    follow_talon.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    // TODO: remember to configure
    lead_talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    lead_talon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);

    follow_talon.follow(lead_talon);
    // TODO: check to see whether inverted or not
    lead_talon.setInverted(true);
    follow_talon.setInverted(true);

    // TODO: Check JVN calculator for current limit
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 10, 25, 0.1);
    lead_talon.configSupplyCurrentLimit(currentLimit);
    follow_talon.configSupplyCurrentLimit(currentLimit);

    // TODO: check whether we need this
    // lead_talon.configOpenloopRamp(0.1);

    // TODO: determine whether we actually want to make this slow
    MotorUtils.setCtreStatusSlow(follow_talon);

    follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 47);
    follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 201);

    // TODO: NEED TO CONFIGURE THESE
    // winch_lead_talon.configForwardSoftLimitThreshold(inchesToTicks(0.0));
    // winch_lead_talon.configForwardSoftLimitEnable(true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (lead_talon.isRevLimitSwitchClosed() == 1) {
      inputs.ElevatorAtLowerLimit = true;
    } else {
      inputs.ElevatorAtLowerLimit = false;
    }

    if (lead_talon.isFwdLimitSwitchClosed() == 1) {
      inputs.ElevatorAtUpperLimit = true;
    } else {
      inputs.ElevatorAtUpperLimit = false;
    }

    inputs.ElevatorTargetHeightInches = targetHeightInches;
    inputs.ElevatorHeightInches =
        ticksToInches(lead_talon.getSensorCollection().getIntegratedSensorPosition());
    inputs.ElevatorAppliedVolts = lead_talon.getMotorOutputVoltage();
    inputs.ElevatorCurrentAmps = new double[] {lead_talon.getSupplyCurrent()};
    inputs.ElevatorTempCelsius = new double[] {lead_talon.getTemperature()};
    // TODO: double check math
    inputs.ElevatorVelocityInchesPerSecond =
        ticks2inches * 10.0 * lead_talon.getSelectedSensorVelocity();
    // TODO: double check math
    inputs.ElevatorVelocityRPM =
        lead_talon.getSelectedSensorVelocity() * 10.0 * ticks2rotations / 60;
  }

  @Override
  public void setMotionProfileConstraints(
      double cruiseVelocityInchesPerSecond, double accelerationInchesPerSecondSquared) {
    // TODO: Use JVN calculator for exact numbers

    double veloTicksPerSecond = inchesToTicks(cruiseVelocityInchesPerSecond);
    double accelTicksPerSecondSquared = inchesToTicks(accelerationInchesPerSecondSquared);

    lead_talon.configMotionCruiseVelocity(veloTicksPerSecond);
    lead_talon.configMotionAcceleration(accelTicksPerSecondSquared);
  }

  private double inchesToTicks(double inches) {
    return inches / ticks2inches;
  }

  private double ticksToInches(double ticks) {
    return ticks * ticks2inches;
  }

  @Override
  public void resetSensorHeight(double heightInches) {
    lead_talon.getSensorCollection().setIntegratedSensorPosition(inchesToTicks(heightInches), 0);
  }

  @Override
  public void setSensorHeightToMax() {
    lead_talon
        .getSensorCollection()
        .setIntegratedSensorPosition(inchesToTicks(maxExtensionInches), 0);
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
}
