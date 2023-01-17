package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControllerRumbleButton extends CommandBase {
  private CommandXboxController controller;
  private double rumbleStrength;
  private boolean rumbleButtonPressed = false;

  /**
   * Rumble the controller until the specified button is pressed
   */
  public ControllerRumbleButton(CommandXboxController controller, JoystickButton rumbleButton, double rumbleStrength) {
    this.controller = controller;
    this.rumbleStrength = rumbleStrength;

    rumbleButton.onTrue(new InstantCommand(() -> {
      rumbleButtonPressed = true;
    }));
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
    return rumbleButtonPressed;
  }
}
