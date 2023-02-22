// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorManualControl extends CommandBase {
  Elevator elevator;
  DoubleSupplier controllerAxis;

  // Flip if using y axis because y is flipped on Xbox controllers
  public ElevatorManualControl(Elevator elevator, DoubleSupplier controllerAxis) {
    this.elevator = elevator;
    this.controllerAxis = controllerAxis;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // negative because up on joystick y axis goes negative
    double elevatorJoyStickValue = controllerAxis.getAsDouble() * 0.2;

    if (Math.abs(elevatorJoyStickValue) > 0.1) {
      elevator.setPercentOutput(elevatorJoyStickValue);
    } else {
      elevator.setPercentOutput(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
