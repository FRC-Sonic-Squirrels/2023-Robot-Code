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

  public Elevator(ElevatorIO io) {
    this.io = io;
    zero();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
  }

  /** Run the Elevator at the specified voltage */
  public void runElevatorVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    io.setElevatorVoltage(voltage * MAX_VOLTAGE);
  }

  public void stop() {
    runElevatorVoltage(0.0);
  }

  public void zero() {
    // TODO: implement zeroing the encoder to height zero
  }

  public void setHeight(double targetHeight) {
    io.setHeight(targetHeight);
  }

  public double getHeight() {
    return io.getHeightInches();
  }

  // TODO: implement methods to get upper and lower limit switch status
  // TODO: implement methods to get elevator motor RPM, velocity in/s
}
