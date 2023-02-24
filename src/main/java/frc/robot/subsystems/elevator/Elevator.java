// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

// Consult the AdvantageKit documentation on how to structure the classes in the elevator directory.
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/CODE-STRUCTURE.md

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  // TODO: limiting to 10V for manual control is a safe starting point
  private double MAX_VOLTAGE = 10.0;
  public static final double toleranceInches = 0.025;
  private boolean zeroed = false;

  public final TunableNumber Kf = new TunableNumber("elevator/Kf", Constants.Elevator.F_CONTROLLER);
  public final TunableNumber Kp = new TunableNumber("elevator/Kp", Constants.Elevator.P_CONTROLLER);
  public final TunableNumber Ki = new TunableNumber("elevator/Ki", Constants.Elevator.I_CONTROLLER);
  public final TunableNumber Kd = new TunableNumber("elevator/Kd", Constants.Elevator.D_CONTROLLER);

  public final TunableNumber cruiseVelocity =
      new TunableNumber(
          "elevator/cruiseVelocity", Constants.Elevator.CRUISE_VELOCITY_INCHES_PER_SEC);
  public final TunableNumber desiredTimeToSpeed =
      new TunableNumber("elevator/desiredTimeToSpeed", Constants.Elevator.DESIRED_TIME_TO_SPEED);

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.resetSensorHeight(0.0);
    setMotionProfileConstraints(cruiseVelocity.get(), desiredTimeToSpeed.get());
    // TODO: load different PIDF values for different IO classes
    io.setPIDConstraints(Kf.get(), Kp.get(), Ki.get(), Kd.get());
    io.setMaxHeightInches(Constants.Elevator.MAX_HEIGHT_INCHES);
    stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    // limit switch
    if (inputs.ElevatorAtLowerLimit) {
      if (!zeroed) {
        // only zero height once per time hitting limit switch
        io.resetSensorHeight(0.0);
        zeroed = true;
      }
    } else {
      // not currently on limit switch, zero again next time we hit limit switch
      zeroed = false;
    }

    // FIXME: add Izone to tuneable parameters
    if (Kf.hasChanged() || Kp.hasChanged() || Ki.hasChanged() || Kd.hasChanged())
      io.setPIDConstraints(Kf.get(), Kp.get(), Ki.get(), Kd.get());

    if (cruiseVelocity.hasChanged() || desiredTimeToSpeed.hasChanged())
      setMotionProfileConstraints(cruiseVelocity.get(), desiredTimeToSpeed.get());

    io.updateProfilePosition();
  }

  /** Run the Elevator at the specified voltage */
  public void runElevatorVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    io.setElevatorVoltage(voltage);
  }

  public void setHeightInches(double targetHeightInches) {
    MathUtil.clamp(targetHeightInches, 0, Constants.Elevator.MAX_HEIGHT_INCHES);

    io.setHeightInches(targetHeightInches);
  }

  public double getHeightInches() {
    return inputs.ElevatorHeightInches;
  }

  /**
   * @return true if withing tolerance of target height
   */
  public boolean isAtHeight(double heightInches) {
    return (Math.abs(heightInches - inputs.ElevatorHeightInches) < toleranceInches);
  }

  /**
   * isAtHeight() check if the elevator is at the target height.
   *
   * @return true if the elevator is at the height setpoint
   */
  public boolean isAtHeight() {
    return isAtHeight(inputs.ElevatorTargetHeightInches);
  }

  /** atLowerLimit() returns true if the lower limit switch is triggered. */
  public boolean atLowerLimit() {
    return (inputs.ElevatorAtLowerLimit);
  }

  /** atUpperLimit() returns true if the upper limit switch is triggered. */
  public boolean atUpperLimit() {
    return (inputs.ElevatorAtUpperLimit);
  }

  public void setPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void stop() {
    io.setPercent(0.0);
  }

  public void zeroHeight() {
    io.resetSensorHeight(0);
  }

  public void setPIDConstraints(double kF, double kP, double kI, double kD) {
    io.setPIDConstraints(kF, kP, kI, kD);
  }

  /**
   * setMotionProfileConstraints() - set the trapezoidal max velocity and acceleration constraints
   * in inches per second.
   *
   * @param cruiseVelocityInchesPerSecond max velocity
   * @param desiredTimeSeconds how long to reach max velocity in seconds
   */
  public void setMotionProfileConstraints(
      double cruiseVelocityInchesPerSecond, double desiredTimeSeconds) {

    double accelerationInchesPerSecondSquared = cruiseVelocityInchesPerSecond / desiredTimeSeconds;

    io.setMotionProfileConstraints(
        cruiseVelocityInchesPerSecond, accelerationInchesPerSecondSquared);
  }
}
