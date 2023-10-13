// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorGoUntilLimitSwitch extends CommandBase {
  /** Creates a new ElevatorGoUntilLimitSwitch. */
  Elevator elevator;

  double speed;

  /**
   * @param elevator
   * @param speed should always be positive
   */
  public ElevatorGoUntilLimitSwitch(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPercentOutput(-Math.abs(speed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atLowerLimit();
  }
}
