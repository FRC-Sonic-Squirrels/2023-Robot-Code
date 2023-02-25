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

  Trigger isFinishedTrigger;

  public ElevatorSetHeight(Elevator elevator, double targetHeightInches) {
    this(elevator, targetHeightInches, 0.1, false, 0.0, 0.0);
  }

  public ElevatorSetHeight(Elevator elevator, double targetHeightInches, double debounceSeconds) {
    this(elevator, targetHeightInches, debounceSeconds, false, 0.0, 0.0);
  }

  public ElevatorSetHeight(
      Elevator elevator,
      double targetHeightInches,
      double motionProfileVelocity,
      double motionProfileDesiredTime) {

    this(elevator, targetHeightInches, 0.1, true, motionProfileVelocity, motionProfileDesiredTime);
  }

  public ElevatorSetHeight(
      Elevator elevator,
      double targetHeightInches,
      double motionProfileVelocity,
      double motionProfileDesiredTime,
      double debounceSeconds) {

    this(
        elevator,
        targetHeightInches,
        debounceSeconds,
        true,
        motionProfileVelocity,
        motionProfileDesiredTime);
  }

  private ElevatorSetHeight(
      Elevator elevator,
      double targetHeightInches,
      double debounce,
      boolean changeMotionProfile,
      double motionProfileVelocity,
      double motionProfileDesiredTime) {

    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;
    this.debounceSeconds = debounce;

    this.changeMotionProfile = changeMotionProfile;
    this.motionProfileVelocity = motionProfileVelocity;
    this.motionProfileDesiredTime = motionProfileDesiredTime;

    this.isFinishedTrigger =
        new Trigger(() -> elevator.isAtHeight(targetHeightInches)).debounce(debounceSeconds);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (changeMotionProfile) {
      System.out.println("CHANGING ELEVATOR PID");
      // elevator.setMotionProfileConstraints(motionProfileVelocity, motionProfileDesiredTime);
    }
    elevator.setHeightInches(targetHeightInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // don't .stop() that disables position control
    // elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO add this back in
    // make sure that the default command kicking in doesnt cause issues
    // return elevator.isAtHeight(targetHeightInches);
    return Math.abs(elevator.getHeightInches() - targetHeightInches) < 0.2;
  }
}
