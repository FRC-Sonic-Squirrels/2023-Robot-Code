package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleTime extends CommandBase {
  private CommandXboxController controller;
  private double rumbleTime;
  private double startTime;
  private double rumbleStrength;

  /**
   * Rumble the controller until the specified time has passed
   * @param rumbleTime Seconds
   */
  public ControllerRumbleTime(CommandXboxController controller, double rumbleTime, double rumbleStrength) {
    this.controller = controller;
    this.rumbleTime = rumbleTime;
    this.rumbleStrength = rumbleStrength;

    this.startTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.getHID().setRumble(RumbleType.kBothRumble, rumbleStrength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= rumbleTime;
  }
}
