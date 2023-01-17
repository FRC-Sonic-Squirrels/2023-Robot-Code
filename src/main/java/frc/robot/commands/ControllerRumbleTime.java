package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleTime extends CommandBase {
  private CommandXboxController controller;
  private long startTime;
  private int rumbleTime;
  private double rumbleStrength;

  /**
   * Rumble the controller until the specified time has passed
   * @param rumbleTime Milliseconds
   */
  public ControllerRumbleTime(CommandXboxController controller, int rumbleTime, double rumbleStrength) {
    this.controller = controller;
    this.rumbleTime = rumbleTime;
    this.rumbleStrength = rumbleStrength;

    this.startTime = System.currentTimeMillis();
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
    return System.currentTimeMillis() - startTime >= rumbleTime;
  }
}
