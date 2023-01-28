package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleInterval extends CommandBase {
  private CommandXboxController controller;
  private double rumbleStrength;
  private int intervals; // number of times to rumble
  private double intervalLength; // the time between each delay
  private double intervalSpace; // the time between each rumble

  private double startRumbling; // time when we started rumbling
  private double stopRumbling;
  private boolean rumbling; // whether we are rumbling or not
  private int cycles = 0; // the number of times we rumbled and then stopped

  /** Rumble the controller n intervals with specified delay, and rumble length */
  public ControllerRumbleInterval(
      CommandXboxController controller,
      int intervals,
      double intervalLength,
      double intervalSpace,
      double rumbleStrength) {
    this.controller = controller;
    this.rumbleStrength = rumbleStrength;

    this.intervals = intervals;
    this.intervalLength = intervalLength;
    this.intervalSpace = intervalSpace;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.getHID().setRumble(RumbleType.kBothRumble, rumbleStrength);

    this.rumbling = true;
    this.startRumbling = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we are rumbling, check if we need to stop rumbling
    if (rumbling) {
      // check if elapsed time since we started rumbling is greater than interavlLength
      if (Timer.getFPGATimestamp() - startRumbling >= intervalLength) {
        // stop rumbling
        rumbling = false;
        stopRumbling = Timer.getFPGATimestamp();
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);

        // we finished a cycle
        cycles += 1;
      }
    } else { // else we are not rumbling, so we need to check if we need to start
      // return if we dont need to start rumbling
      if (Timer.getFPGATimestamp() - stopRumbling >= intervalSpace) {
        rumbling = true;
        startRumbling = Timer.getFPGATimestamp();
        controller.getHID().setRumble(RumbleType.kBothRumble, rumbleStrength);
      }
    }

    // SmartDashboard.putBoolean("rumble", rumbling);
    // SmartDashboard.putNumber("cycle", cycles);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    // SmartDashboard.putBoolean("rumble", false);

    cycles = 0;
    // SmartDashboard.putNumber("cycle", cycles);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return whether cycles is greater or equal to intervals
    return cycles >= intervals;
  }
}
