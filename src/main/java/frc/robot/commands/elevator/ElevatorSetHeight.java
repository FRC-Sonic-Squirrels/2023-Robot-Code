// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorSetHeight extends CommandBase {
  /** Creates a new ElevatorSetHeight. */
  Elevator elevator;

  double targetHeightInches;
  boolean changeMotionProfile = false;

  double motionProfileVelocity;
  double motionProfileDesiredTime;

  double debounceSeconds = 0.1;

  public ElevatorSetHeight(Elevator elevator, double targetHeightInches) {
    changeMotionProfile = false;

    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ElevatorSetHeight(Elevator elevator, double targetHeightInches, double debounceSeconds) {
    changeMotionProfile = false;

    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;

    this.debounceSeconds = debounceSeconds;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ElevatorSetHeight(
      Elevator elevator,
      double targetHeightInches,
      double motionProfileVelocity,
      double motionProfileDesiredTime) {

    changeMotionProfile = true;
    this.motionProfileVelocity = motionProfileVelocity;
    this.motionProfileDesiredTime = motionProfileDesiredTime;

    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ElevatorSetHeight(
      Elevator elevator,
      double targetHeightInches,
      double motionProfileVelocity,
      double motionProfileDesiredTime,
      double debounceSeconds) {

    changeMotionProfile = true;
    this.motionProfileVelocity = motionProfileVelocity;
    this.motionProfileDesiredTime = motionProfileDesiredTime;

    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;
    this.debounceSeconds = debounceSeconds;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (changeMotionProfile) {
      elevator.setMotionProfileConstraints(motionProfileVelocity, motionProfileDesiredTime);
    }
    elevator.setHeightInches(targetHeightInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // elevator has to be at that height for 0.1 seconds
    // TODO: check if this is needed
    return new Trigger(() -> elevator.isAtHeight(targetHeightInches))
        .debounce(debounceSeconds)
        .getAsBoolean();
  }
}
