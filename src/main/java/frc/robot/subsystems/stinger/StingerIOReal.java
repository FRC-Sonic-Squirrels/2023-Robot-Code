// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIVOR_canId;

/** Add your docs here. */
public class StingerIOReal implements StingerIO{

    // The stinger has similar code to the elevator, but it only has 1 motor and no brake (and it is horizontal)
    private WPI_TalonFX winchTalon =
        new WPI_TalonFX(CANIVOR_canId.CANID5_STINGER_TALON, CANIVOR_canId.name);

    //FIXME: these are the elevator values. find out the stinger ones
    private final double gearRatio = 0.08229; // 0.074;
    // TODO: note diameter is set to 1.9 on JVN
    private final double winchDiameter_inches = 1.43; // 1.25 diameter + string windings
    private final double winchCircumference = Math.PI * winchDiameter_inches;
    private final double maxExtensionInches = 26; // actually 24 letting more for unwinding
    private double lengthSetpointInches = 0.0;
    private double feedForwardClimbing = 0.025734; // from JVM calculator
    private double feedForwardDescending = 0.0257; // 0.001;
    private final double ticks2distance = gearRatio * winchCircumference / 2048;
    //TODO make sure conversion is correct
    private final double ticks2rotation = 1/4096;
    private boolean zeroed = false;

    public boolean m_atMaxLength;

    public double m_currentLength;
    public double targetLengthInches;

    public StingerIOReal() {
        winchTalon.configFactoryDefault();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor =
            TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        
        // Details on elevator motors, gearing and calculated kP and kFF are here
        // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
        // this also has suggest trapezoidal velocity profile constants.
        //TODO: get the right PID values for the stinger
        setPIDConstraints(0.054, 0.48, 0.0, 0.0);

        // do we need this if the command is updating the motion magic constraints?
        // maybe have a safe default?
        config.motionAcceleration = 30000; // 60941;    //  20521 ticks/100ms     = 11 in/s
        config.motionCruiseVelocity = 15235; //  20521 ticks/100ms/sec = 11 in/s^2

        config.slot0.allowableClosedloopError = Stinger.toleranceInches / ticks2distance;

        // set config
        winchTalon.configAllSettings(config);

        // use pid from slot 0 for motion magic
        winchTalon.selectProfileSlot(0, 0);

        winchTalon.setNeutralMode(NeutralMode.Brake);

        // config hard limit switch for full down position
        winchTalon.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        winchTalon.setInverted(false);

        // JVN calculator predicts 41.2 A per motor under load
        // TODO: Check new JVN prediction
        SupplyCurrentLimitConfiguration currentLimit =
            new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
        winchTalon.configSupplyCurrentLimit(currentLimit);

        winchTalon.configOpenloopRamp(0.1);

        setSensorPosition(0.0);

        winchTalon.configReverseSoftLimitThreshold(lengthToTicks(maxExtensionInches));
        winchTalon.configReverseSoftLimitEnable(true);
    }


    public void setStingerVoltage(double volts) {
        winchTalon.setVoltage(volts);
    }


    public void setPercent(double percent) {
        winchTalon.set(ControlMode.PercentOutput, percent);
    }


    @Override
    public void setExtensionInches(double heightInches) {
        // if (heightInches < 0.0) {
        //   heightInches = 0.0;
        // }
        if (heightInches > maxExtensionInches) {
        heightInches = maxExtensionInches;
        }

        winchTalon.set(ControlMode.MotionMagic, lengthToTicks(heightInches));
    }


    /**
    * @param acceleration accel in inches per second^2
    * @param cruiseVelocity max velocity in inches per second
    */
    @Override
    public void setMotionMagicConstraints(double cruiseVelocity, double acceleration) {
        // math adapted from howdybots jvn calculator equation
        //TODO: if speed and acceleration are strange during testing, check to see if these values are correct
        double veloInTicks = cruiseVelocity * (12.15 / winchCircumference) * 2048 / 10;
        double accelInTicks = acceleration * (12.15 / winchCircumference) * 2048 / 10;

        winchTalon.configMotionAcceleration(accelInTicks);
        winchTalon.configMotionCruiseVelocity(veloInTicks);

        // temporary for debugging
        SmartDashboard.putNumber("Stinger MM Constraint acceleration", acceleration);
        SmartDashboard.putNumber("Stinger MM Constraint velo INCHES", cruiseVelocity);

        SmartDashboard.putNumber("Stinger MM Constraint accel TICKS", accelInTicks);
        SmartDashboard.putNumber("Stinger MM Constraint velo TICKS", veloInTicks);
    }


    public void resetSensorPosition(double position) {
        winchTalon.getSensorCollection().setIntegratedSensorPosition(position, 0);
    }


    public void setPIDConstraints(double feedForward, double kP, double kI, double kD) {
        winchTalon.config_kP(0, kP);
        winchTalon.config_kF(0, feedForward);
        winchTalon.config_kI(0, kI);
        winchTalon.config_kD(0, kD);
        winchTalon.config_IntegralZone(0, 0);
        winchTalon.configClosedLoopPeakOutput(0, 1);
    }


    public double lengthToTicks(double lengthInches) {
        return lengthInches / ticks2distance;
    }


    public double ticksToLength(double ticks) {
        return ticks * ticks2distance;
    }

}
