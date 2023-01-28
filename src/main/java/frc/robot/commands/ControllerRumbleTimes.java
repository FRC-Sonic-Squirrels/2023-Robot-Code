// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleTimes extends CommandBase {
  private CommandXboxController controller;
  private double rumbleStrength;
  private double numberofTimes;
  private double timeBetweenRumbles;
  private double uptimeTimer;
  private double downtimeTimer;
  private double timeToRumble;
  private boolean isRumbling;

  /** Creates a new ControllerRumbleTimes. */
  public ControllerRumbleTimes(
      CommandXboxController controller,
      double rumbleStrength,
      double numberofTimes,
      double timeBetweenRumblesSeconds,
      double timeToRumbleSeconds) {
    this.controller = controller;
    this.rumbleStrength = rumbleStrength;
    this.numberofTimes = numberofTimes;
    this.timeBetweenRumbles = timeBetweenRumblesSeconds;
    this.timeToRumble = timeToRumbleSeconds;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRumbling = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // start time gets set the first time u go in rumble
    if (isRumbling == false) {
      if (Timer.getFPGATimestamp() - downtimeTimer >= timeBetweenRumbles) {
        controller.getHID().setRumble(RumbleType.kBothRumble, rumbleStrength);
        isRumbling = true;
        uptimeTimer = Timer.getFPGATimestamp();
      }
    }

    if (Timer.getFPGATimestamp() - uptimeTimer >= timeToRumble) {
      isRumbling = false;
      downtimeTimer = Timer.getFPGATimestamp();
      controller.getHID().setRumble(RumbleType.kBothRumble, 0);
      numberofTimes--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numberofTimes == 0;
  }
}
