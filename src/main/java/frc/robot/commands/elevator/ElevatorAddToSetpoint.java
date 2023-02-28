// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.BooleanSupplier;

public class ElevatorAddToSetpoint extends CommandBase {
  /** Creates a new ElevatorAddToSetpoint. */
  Elevator elevator;

  double offset;
  BooleanSupplier buttonPress;
  boolean moving;

  double newTargetHeight;

  public ElevatorAddToSetpoint(Elevator elevator, double offset, BooleanSupplier buttonPress) {
    this.elevator = elevator;
    this.offset = offset;
    this.buttonPress = buttonPress;

    this.moving = false;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeight = elevator.getHeightInches();

    if (buttonPress.getAsBoolean() && !moving) {
      elevator.setHeightInches(currentHeight + offset);
      moving = true;
      newTargetHeight = currentHeight + offset;
    }

    if (elevator.isAtHeight(newTargetHeight)) {
      moving = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
