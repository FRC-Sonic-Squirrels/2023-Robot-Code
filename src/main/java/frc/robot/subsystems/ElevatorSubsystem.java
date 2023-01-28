// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  // FIXME: find actual falcon and solenoid device numbers
  private WPI_TalonFX winchLeadTalon = new WPI_TalonFX(-1);
  private WPI_TalonFX winchFollowTalon = new WPI_TalonFX(-2);

  // Unsure if we need a friction brake solenoid, including one just in case
  private Solenoid frictionBrakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, -3);

  // FIXME: find actual number measurements, most of these are placeholders
  // TODO: find the number of gear teeth (ticks) required to move the chain an inch
  private final double winchDiameterInches = 0;
  private final double winchCircumference = 0;
  private final double gearRatio = 1.0 / 6.0;
  private final double ticksPerInch = 0;

  private final double max_extension_inches = 0;
  private double set_point_inches = 0;
  private double tolerance_inches = 0.05;
  private double feed_forward_climbing = 0; // get from JVM calculator
  private double feed_forward_descending = 0;

  private boolean zeroed = false;

  public boolean m_atMaxHeight;
  public double m_currentHeight;

  /** Creates a new ElevatorSubsystem.
  public ElevatorSubsystem() {

    winchLeadTalon.configFactoryDefault();
    winchFollowTalon.configFactoryDefault();

    TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    leadConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/13KUbQWcU_HbGfyZCWiUc_FifyZQka65Ql4cd2lHTFQk/edit#gid=57306390
    // this also has suggest trapezoidal velocity profile constants.
    // TODO: see if these PID values need to be changed
    leadConfig.slot0.kF = 0.054;
    leadConfig.slot0.kP = 0.48; // 0.054836;
    leadConfig.slot0.kI = 0.0;
    leadConfig.slot0.kD = 0.0;
    leadConfig.slot0.integralZone = 0.0;
    leadConfig.slot0.closedLoopPeakOutput = 1.0;

    // TODO: see if this command is needed even with motion magic constraints. Maybe have a safe
    // default?
    leadConfig.motionAcceleration = 30000; // 60941;  //  20521 ticks/100ms     = 11 in/s
    leadConfig.motionCruiseVelocity = 15235; //  20521 ticks/100ms/sec = 11 in/s^2

    winchLeadTalon.configAllSettings(leadConfig);

    // use pid from slot 0 for motion magic
    winchLeadTalon.selectProfileSlot(0, 0);

    winchLeadTalon.setNeutralMode(NeutralMode.Brake);
    winchFollowTalon.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    winchLeadTalon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    winchFollowTalon.follow(winchLeadTalon);
    winchLeadTalon.setInverted(false);
    winchFollowTalon.setInverted(false);

    // JVN Motor currently predicts 56.51 amps per motor under load
    // TODO: find new measurements based on new values
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
    winchLeadTalon.configSupplyCurrentLimit(currentLimit);
    winchFollowTalon.configSupplyCurrentLimit(currentLimit);

    // stop throttle from going from 0 to full too fast (must take 0.1 seconds minimum)
    winchLeadTalon.configOpenloopRamp(0.1);

    // reduce CAN traffic whenever possible
    // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html
    // TODO: add MotorUtils or an equivalent to this project to reduce CAN traffic in
    // winchFollowTalon

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
*/
