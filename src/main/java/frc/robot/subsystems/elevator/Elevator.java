// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

// Consult the AdvantageKit documentation on how to structure the classes in the elevator directory.
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/CODE-STRUCTURE.md

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private double MAX_VOLTAGE = 10.0;
  public static double toleranceInches = 0.05;
  private boolean zeroed;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.zeroHeight();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    if (inputs.ElevatorAtLowerLimit) {
      if (!zeroed) {
        // only zero height once per time hitting limit switch
        io.zeroHeight();
        zeroed = true;
      }
    } else {
      // not currently on limit switch, zero again next time we hit limit switch
      zeroed = false;
    }
  }

  /** Run the Elevator at the specified voltage */
  public void runElevatorVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    io.setElevatorVoltage(voltage * MAX_VOLTAGE);
  }

  public void stop() {
    runElevatorVoltage(0.0);
  }

  public void setHeightInches(double targetHeightInches) {
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
    return isAtHeight(inputs.ElevatorHeightInches);
  }

  /** atLowerLimit() returns true if the lower limit switch is triggered. */
  public boolean atLowerLimit() {
    return (inputs.ElevatorAtLowerLimit);
  }

  public void brakeOn() {
    io.brakeOn();
  }

  public void brakeOff() {
    io.brakeOff();
  }

  public void setWinchPercentOutput(double percent) {
    io.setPercent(percent);
  }

  // TODO: implement methods to get upper and lower limit switch status
  // TODO: implement methods to get elevator motor RPM, velocity in/s
}
