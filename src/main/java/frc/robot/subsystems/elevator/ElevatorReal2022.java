// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team2930.lib.util.MotorUtils;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import org.littletonrobotics.junction.Logger;

// Details on the TalonFX motion profile control can be found here:
// https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
// Example code:
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop_AuxFeedForward/src/main/java/frc/robot/Robot.java

public class ElevatorReal2022 implements ElevatorIO {
  // HACK: manually set correct canbus name for 2022 bot
  private WPI_TalonFX lead_talon = new WPI_TalonFX(CanId.CANID9_ELEVATOR_LEAD_TALON, "CANivore");
  private WPI_TalonFX follow_talon =
      new WPI_TalonFX(CanId.CANID10_ELEVATOR_FOLLOW_TALON, "CANivore");
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.pneumatics.channel_15_friction_brake);
  private final double gearRatio = 0.08229; // 0.074;
  // TODO: note diameter is set to 1.9 on JVN
  private final double winchDiameter_inches = 1.43; // 1.25 diameter + string windings
  private final double winchCircumference = Math.PI * winchDiameter_inches;
  private final double maxExtensionInches = 26; // actually 24 letting more for unwinding
  private double heightSetpointInches = 0.0;
  private double feedForwardClimbing = 0.025734; // from JVM calculator
  private double feedForwardDescending = 0.0257; // 0.001;
  private final double ticks2distance = gearRatio * winchCircumference / 2048;

  private final double ticks2rotation = 1 / 2048;
  private boolean zeroed = false;

  public boolean m_atMaxheight;

  private double m_currentHeight;

  private double targetHeightInches;

  // state of solenoids when active
  private boolean solenoidEnabled = false;

  private final TunableNumber Kp =
      new TunableNumber("Elevator/Kp", Constants.Elevator2022.P_CONTROLLER);
  private final TunableNumber Ki =
      new TunableNumber("Elevator/Ki", Constants.Elevator2022.I_CONTROLLER);
  private final TunableNumber Kd =
      new TunableNumber("Elevator/Kd", Constants.Elevator2022.D_CONTROLLER);

  public ElevatorReal2022() {
    lead_talon.configFactoryDefault();
    follow_talon.configFactoryDefault();

    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggest trapezoidal velocity profile constants.
    leadConfig.slot0.kF = 0.054;
    leadConfig.slot0.kP = Kp.get(); // 0.054836;
    leadConfig.slot0.kI = Ki.get();
    leadConfig.slot0.kD = Kd.get();
    leadConfig.slot0.integralZone = 0.0;
    leadConfig.slot0.closedLoopPeakOutput = 1.0;

    // do we need this if the command is updating the motion magic constraints?
    // maybe have a safe default?
    leadConfig.motionAcceleration = 30000; // 60941;    //  20521 ticks/100ms     = 11 in/s
    leadConfig.motionCruiseVelocity = 15235; //  20521 ticks/100ms/sec = 11 in/s^2

    leadConfig.slot0.allowableClosedloopError = Elevator.toleranceInches / ticks2distance;

    // set config
    lead_talon.configAllSettings(leadConfig);

    // configure integrated sensor as selected sensor
    lead_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // use pid from slot 0 for motion magic
    lead_talon.selectProfileSlot(0, 0);

    lead_talon.setNeutralMode(NeutralMode.Brake);
    follow_talon.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    lead_talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);
    lead_talon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);

    follow_talon.follow(lead_talon);
    lead_talon.setInverted(true);
    follow_talon.setInverted(true);

    // JVN calculator predicts 41.2 A per motor under load

    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
    lead_talon.configSupplyCurrentLimit(currentLimit);
    follow_talon.configSupplyCurrentLimit(currentLimit);

    // Open loop effects manual control only
    lead_talon.configOpenloopRamp(0.1);

    // Reduce CAN traffic where possible
    // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html
    MotorUtils.setCtreStatusSlow(follow_talon);
    // leave lead motor with default CAN settings. We need position and limit switch updates

    // NOTE: when we power up, we expect the elevator to be full down, triggering the lower limit
    // switch.
    // if not, we need to move the elevator down to the lower limit switch (VERY SLOWLY).
    // hitting either limit switch must stop the elevator.

    // Stagger update frames to reduce congestion
    follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 47);
    follow_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 201);

    brakeOn();
    resetSensorHeight(0.0);

    // // set soft limit on forward movement (down)
    // winch_lead_talon.configForwardSoftLimitThreshold(heightToTicks(0.0));
    // winch_lead_talon.configForwardSoftLimitEnable(true);

    // set soft limit on reverse movement (Up)
    lead_talon.configForwardSoftLimitThreshold(inchesToTicks(maxExtensionInches));
    lead_talon.configForwardSoftLimitEnable(true);
  }

  /**
   * @param acceleration accel in inches per second^2
   * @param cruiseVelocity max velocity in inches per second
   */
  @Override
  public void setMotionProfileConstraints(double cruiseVelocity, double desiredTimeToSpeed) {
    // math adapted from howdybots jvn calculator equation
    double veloInTicks = cruiseVelocity * (12.15 / winchCircumference) * 2048 / 10;
    double accelInTicks = veloInTicks / desiredTimeToSpeed;

    lead_talon.configMotionAcceleration(accelInTicks);
    lead_talon.configMotionCruiseVelocity(veloInTicks);

    // temporary for debugging
    SmartDashboard.putNumber("Elevator MM Constraint desiredTimeToSpeed", desiredTimeToSpeed);
    SmartDashboard.putNumber("Elevator MM Constraint velo INCHES", cruiseVelocity);

    SmartDashboard.putNumber("Elevator MM Constraint accel TICKS", accelInTicks);
    SmartDashboard.putNumber("Elevator MM Constraint velo TICKS", veloInTicks);
  }

  public void setMotionMagicSetPoint(double heightInches) {
    // if (heightInches < 0.0) {
    //   heightInches = 0.0;
    // }
    if (heightInches > maxExtensionInches) {
      heightInches = maxExtensionInches;
    }

    lead_talon.set(ControlMode.MotionMagic, inchesToTicks(heightInches));

    // if (heightInches <= heightSetpointInches) {
    //   // lifting up robot, use more feed forward
    //   winch_lead_talon.set(TalonFXControlMode.MotionMagic, heightToTicks(heightInches),
    //       DemandType.ArbitraryFeedForward, feedForwardClimbing);
    // } else {
    //   // lowering robot, use less feed forward
    //   winch_lead_talon.set(TalonFXControlMode.MotionMagic, heightToTicks(heightInches),
    //       DemandType.ArbitraryFeedForward, feedForwardDescending);
    // }

    heightSetpointInches = heightInches;

    SmartDashboard.putNumber("Elevator MM Height SetPoint", heightInches);
  }

  @Override
  public void setHeightInches(double targetHeightInches) {
    if (targetHeightInches < 0.0) {
      targetHeightInches = 0.0;
    }
    if (targetHeightInches > maxExtensionInches) {
      targetHeightInches = maxExtensionInches;
    }

    if (targetHeightInches <= heightSetpointInches) {
      // lifting up robot, use more feed forward
      lead_talon.set(
          TalonFXControlMode.Position,
          inchesToTicks(targetHeightInches),
          DemandType.ArbitraryFeedForward,
          feedForwardClimbing);
    } else {
      // lowering robot, use less feed forward
      lead_talon.set(
          TalonFXControlMode.Position,
          inchesToTicks(targetHeightInches),
          DemandType.ArbitraryFeedForward,
          feedForwardDescending);
    }
    // NOTE: this is how without arbitrary feed forward
    // winch_lead_talon.set(TalonFXControlMode.Position, heightToTicks(heightInches));

    heightSetpointInches = targetHeightInches;
  }

  @Override
  public void resetSensorHeight(double heightInches) {
    lead_talon.getSensorCollection().setIntegratedSensorPosition(inchesToTicks(heightInches), 0);
  }

  private double inchesToTicks(double heightInches) {
    return heightInches / ticks2distance;
  }

  private double ticksToInches(double ticks) {
    return ticks * ticks2distance;
  }

  /**
   * getHeightInches() returns the current height of the elevator in inches.
   *
   * @return height of the elevator in inches
   */

  /**
   * zeroHeight() resets the starting position of the elevator to the current height.
   *
   * <p>This is called when the elevator triggers the lower limit switch. This needs to be done by a
   * command, that runs the elevator to the lower limit switch. VERY SLOWLY.
   */

  /** Manually run elevator motors. USE WITH CAUTION. */
  @Override
  public void setPercent(double percent) {
    lead_talon.set(ControlMode.PercentOutput, percent);
  }

  /** brakeOn() turns on the brake. */
  @Override
  public void brakeOff() {
    frictionBrakeSolenoid.set(!solenoidEnabled);
  }

  /** brakeOff() turns off the brake. */
  @Override
  public void brakeOn() {
    frictionBrakeSolenoid.set(solenoidEnabled);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    lead_talon.setVoltage(volts);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (lead_talon.isRevLimitSwitchClosed() == 1) {
      inputs.ElevatorAtLowerLimit = true;
    } else {
      inputs.ElevatorAtLowerLimit = false;
    }

    if (inputs.ElevatorHeightInches >= maxExtensionInches) {
      m_atMaxheight = true;
      inputs.ElevatorAtUpperLimit = true;
    } else {
      m_atMaxheight = false;
      inputs.ElevatorAtUpperLimit = false;
    }

    // cache position and velocity values
    double sensorPosition = lead_talon.getSelectedSensorPosition();
    double sensorVelocity = lead_talon.getSelectedSensorVelocity();

    inputs.ElevatorTargetHeightInches = targetHeightInches;
    inputs.ElevatorHeightInches = ticksToInches(sensorPosition);

    inputs.ElevatorAppliedVolts = lead_talon.getMotorOutputVoltage();
    inputs.ElevatorCurrentAmps = new double[] {lead_talon.getSupplyCurrent()};
    inputs.ElevatorTempCelsius = new double[] {lead_talon.getTemperature()};
    inputs.ElevatorVelocityInchesPerSecond = ticks2distance * 10.0 * sensorVelocity;
    inputs.ElevatorVelocityRPM = sensorVelocity * 10.0 * ticks2rotation;

    Logger.getInstance().recordOutput("Elevator/ElevatorUpperSoftLimit", maxExtensionInches);
    Logger.getInstance().recordOutput("Elevator/ElevatorHeightTicks", sensorPosition);
    Logger.getInstance()
        .recordOutput("Elevator/ElevatorUpperSoftLimitTicks", inchesToTicks(maxExtensionInches));
  }

  // @Override
  public void periodic() {

    // check if we triggered lower limit switch, and reset elevator to zero

    // SmartDashboard.putBoolean("ELEVATOR AT THE MAX HEIGHT", m_atMaxheight);

    // // if(this.getCurrentCommand() != null){
    // //   SmartDashboard.putString("AAA elevator current command",
    // // this.getCurrentCommand().toString());
    // // } else {
    // //   SmartDashboard.putString("AAA elevator current command", "null");
    // // }

    // SmartDashboard.putNumber("Elevator height ticks", getHeightTicks());
    // SmartDashboard.putNumber("Elevator Height (inches)", getHeightInches());
    // SmartDashboard.putNumber("Elevator Height Set Point", heightSetpointInches);
    // // SmartDashboard.putNumber("Elevator Height (ticks)", getHeightTicks());
    // SmartDashboard.putNumber(
    //     "Elevator current Vel (inches per s)",
    //     ticks2distance * 10.0 * winch_lead_talon.getSelectedSensorVelocity());
    // SmartDashboard.putNumber(
    //     "Elevator current vel ticks", winch_lead_talon.getSelectedSensorVelocity() * 10.0);
    // // SmartDashboard.putNumber("Elevator SetPoint inches", heightSetpointInches);
    // // SmartDashboard.putNumber("Elevator SetPoint (ticks)",
    // heightToTicks(heightSetpointInches));
    // SmartDashboard.putNumber("Elevator Error", heightSetpointInches - getHeightInches());
    // SmartDashboard.putBoolean("Elevator limit", atLowerLimit());
    // SmartDashboard.putNumber("Elevator %output", winch_lead_talon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Elevator Current Lead", winch_lead_talon.getSupplyCurrent());
    // SmartDashboard.putNumber("Elevator Current Follow", winch_follow_talon.getSupplyCurrent());
    // SmartDashboard.putBoolean("Elevator Brake On", !frictionBrakeSolenoid.get());

    // // debug values for MM. These should match the values from setMotionMagicConstraints()
    // // note: These along with all the other config options are viewable and editable from phoenix
    // // tuner
    // SmartDashboard.putNumber(
    //     "Elevator MM config acceleration",
    //     (winch_lead_talon.configGetParameter(ParamEnum.eMotMag_Accel, 0) * ticks2distance * 10));
    // SmartDashboard.putNumber(
    //     "Elevator MM config velocity ",
    //     (winch_lead_talon.configGetParameter(ParamEnum.eMotMag_VelCruise, 0)
    //         * ticks2distance
    //         * 10));
    // SmartDashboard.putNumber(
    //     "Elevator MM config S-curve",
    //     winch_lead_talon.configGetParameter(ParamEnum.eMotMag_SCurveLevel, 0));

    // SmartDashboard.updateValues();
  }
}
