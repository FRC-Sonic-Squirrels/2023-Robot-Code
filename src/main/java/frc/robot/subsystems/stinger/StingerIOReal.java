// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import org.littletonrobotics.junction.Logger;

public class StingerIOReal implements StingerIO {

  // The stinger has similar code to the elevator, but it only has 1 motor and no brake (and it is
  // horizontal)
  private WPI_TalonFX motor = new WPI_TalonFX(CanId.CANID5_STINGER_TALON, CanId.name);
  private final double gearRatio = 0.144; // (12 * 30) / (50 * 50)
  private final double pulleyDiameterInches = 1.125;
  private final double pulleyCircumference = pulleyDiameterInches * Math.PI;
  // Multiplied by 2 because of the continuous elevator; double the elevator moves
  private final double cascadeMultiplier = 2.0;
  private double ticks2Inches = gearRatio * pulleyCircumference * cascadeMultiplier / 2048.0;
  private static final double ticks2rotations = 1.0 / 2048.0;

  private final double maxExtensionInches = Constants.Stinger.MAX_EXTENSION_INCHES;
  private double targetExtensionInches = 0.0;

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.Stinger.CRUISE_VELOCITY_INCHES_PER_SEC,
          (Constants.Stinger.CRUISE_VELOCITY_INCHES_PER_SEC
              / Constants.Stinger.DESIRED_TIME_TO_SPEED));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile profile = new TrapezoidProfile(constraints, goal);

  private boolean positionControlMode = true;

  private double lastCloseLoopExecutionTime = -1.0;

  public StingerIOReal() {
    motor.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // JVN calculator w/ suggested kFF, kP, velocity, and acceleration
    // https://ss2930.sharepoint.com/:x:/r/sites/Programming/_layouts/15/Doc.aspx?sourcedoc=%7B318D8C0F-AC95-43F3-B4DB-0964BE9A2FD1%7D&file=elevator%202023%20howdybots%20version.xlsx&action=default&mobileredirect=true

    config.slot0.allowableClosedloopError = Stinger.toleranceInches / ticks2Inches;
    // we don't use I why is this here? also it should live in Stinger top level in
    // setPIDConstraints()
    config.slot0.integralZone = inchesToTicks(0.2);

    // set config
    motor.configAllSettings(config);

    // configure integrated sensor as selected sensor
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // use pid from slot 0 for motion magic
    motor.selectProfileSlot(0, 0);

    // FIXME: set to Brake mode after testing
    motor.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    motor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    motor.setInverted(true);

    // JVN calculator predicts 12 A under load
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
    motor.configSupplyCurrentLimit(currentLimit);

    // this only effects manual control
    motor.configOpenloopRamp(0.1);

    setSensorPosition(0.0);

    // TODO: look into increasing our stinger reach to place those super sucked in cones a bit
    // easier
    motor.configForwardSoftLimitThreshold(inchesToTicks(maxExtensionInches));
    motor.configForwardSoftLimitEnable(true);
  }

  @Override
  public void updateInputs(StingerIOInputs inputs) {
    if (motor.isRevLimitSwitchClosed() == 1) {
      inputs.StingerAtRetractedLimit = true;
    } else {
      inputs.StingerAtRetractedLimit = false;
    }

    inputs.StingerTargetExtensionInches = targetExtensionInches;
    inputs.StingerExtensionInches = ticksToInches(motor.getSelectedSensorPosition());
    inputs.StingerAtExtendedLimit = (inputs.StingerExtensionInches >= maxExtensionInches);
    inputs.StingerSetpointInches = setpoint.position;

    inputs.StingerAppliedVolts = motor.getMotorOutputVoltage();
    inputs.StingerCurrentAmps = new double[] {motor.getSupplyCurrent()};
    inputs.StingerTempCelsius = new double[] {motor.getTemperature()};
    // NOTE: don't use talon.getSensorCollection() see:
    // https://www.chiefdelphi.com/t/swerve-odometry-problems/392680/5
    double sensorVelocity = motor.getSelectedSensorVelocity();
    inputs.StingerVelocityInchesPerSecond = ticksToInchesPerSecond(sensorVelocity);
    inputs.StingerVelocityRPM = sensorVelocity * 10.0 * ticks2rotations / 60.0;

    Logger.getInstance().recordOutput("Stinger/distanceInTicks", motor.getSelectedSensorPosition());

    Logger.getInstance()
        .recordOutput(
            "Stinger/distanceError", inputs.StingerSetpointInches - inputs.StingerExtensionInches);
  }

  public void updateProfilePosition() {
    // if (positionControlMode) {
    //   if ((targetExtensionInches <= 0.0) && (motor.isRevLimitSwitchClosed() == 1)) {
    //     // we are on the limit switch and target is zero height, turn off motor
    //     positionControlMode = false;
    //     setPercent(0.0);
    //     return;
    //   }

    //   // update profile
    //   profile = new TrapezoidProfile(constraints, goal, setpoint);

    //   setpoint = profile.calculate(0.02);

    //   motor.set(
    //       TalonFXControlMode.Position,
    //       inchesToTicks(setpoint.position),
    //       DemandType.ArbitraryFeedForward,
    //       Constants.Stinger.ARBITRARY_FEED_FORWARD);
    // }

    if (positionControlMode) {
      if ((targetExtensionInches <= 0.0) && (motor.isRevLimitSwitchClosed() == 1)) {
        // we are on the limit switch and target is zero height, turn off motor
        positionControlMode = false;
        setPercent(0.0);
        return;
      }

      if (lastCloseLoopExecutionTime == -1.0) {
        lastCloseLoopExecutionTime = Timer.getFPGATimestamp();
      }

      // update profile
      profile = new TrapezoidProfile(constraints, goal, setpoint);

      var currentTime = Timer.getFPGATimestamp();

      Logger.getInstance()
          .recordOutput("Stinger/lastClosedLoopExecutionTime", lastCloseLoopExecutionTime);

      setpoint = profile.calculate(currentTime - lastCloseLoopExecutionTime);

      motor.set(
          TalonFXControlMode.Position,
          inchesToTicks(setpoint.position),
          DemandType.ArbitraryFeedForward,
          Constants.Elevator.ARBITRARY_FEED_FORWARD);

      lastCloseLoopExecutionTime = currentTime;
    } else {
      targetExtensionInches = 0.0;
      lastCloseLoopExecutionTime = -1.0;
    }
  }

  public void setStingerVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void setPercent(double percent) {
    positionControlMode = false;
    lastCloseLoopExecutionTime = -1.0;
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setExtensionInches(double extensionInches) {
    // if (extensionInches < 0.0) {
    //   extensionInches = 0.0;
    // } else if (extensionInches > maxExtensionInches) {
    //   extensionInches = maxExtensionInches;
    // }
    // targetExtensionInches = extensionInches;
    // motor.set(ControlMode.MotionMagic, inchesToTicks(extensionInches));

    this.targetExtensionInches = extensionInches;
    positionControlMode = true;
    goal = new TrapezoidProfile.State(targetExtensionInches, 0);

    // starting setpoint is our current position and velocity
    setpoint =
        new TrapezoidProfile.State(
            ticksToInches(motor.getSelectedSensorPosition()),
            ticksToInchesPerSecond(motor.getSelectedSensorVelocity()));

    updateProfilePosition();
  }

  /**
   * @param acceleration accel in inches per second^2
   * @param cruiseVelocity max velocity in inches per second
   */
  @Override
  public void setMotionProfileConstraints(
      double cruiseVelocityInchesPerSecond, double accelerationInchesPerSecondSquared) {

    constraints =
        new TrapezoidProfile.Constraints(
            cruiseVelocityInchesPerSecond, accelerationInchesPerSecondSquared);
  }

  @Override
  public void setSensorPosition(double position) {
    motor.setSelectedSensorPosition(position);
  }

  @Override
  public void setPIDConstraints(double feedForward, double kP, double kI, double kD) {

    motor.config_kF(0, feedForward);
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    // motor.config_IntegralZone(0, 0);
    // motor.configClosedLoopPeakOutput(0, 1);
  }

  public double inchesToTicks(double extensionInches) {
    return extensionInches / ticks2Inches;
  }

  public double ticksToInches(double ticks) {
    return ticks * ticks2Inches;
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
}
