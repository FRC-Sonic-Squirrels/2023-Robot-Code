// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorControlCommand extends CommandBase {
  Elevator elevator;
  CommandXboxController controller;
  double gain = 1.0;

  public ElevatorControlCommand(Elevator elevator, CommandXboxController controller, double gain) {
    this.elevator = elevator;
    this.controller = controller;
    this.gain = gain;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // negative because up on joystick y axis goes negative
    double elevatorJoyStickValue = -controller.getLeftY();

    if (Math.abs(elevatorJoyStickValue) > 0.1) {
      elevator.brakeOff();
      elevator.setWinchPercentOutput(elevatorJoyStickValue * gain);
    } else {
      elevator.setWinchPercentOutput(0.0);
      elevator.brakeOn();
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