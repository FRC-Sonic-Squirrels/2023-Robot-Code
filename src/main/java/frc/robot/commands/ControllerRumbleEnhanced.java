package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;

public class ControllerRumbleEnhanced extends CommandBase {
  private CommandXboxController controller;
  private double rumbleStrength;

  private double startTime;
  private double rumbleTime;

  private BooleanSupplier rumbleSupplier;

  /**
   * Rumble the controller until either the button is pressed or the specified time is passed
   *
   * @param controller
   * @param rumbleButton
   * @param rumbleTime
   * @param rumbleStrength
   */
  public ControllerRumbleEnhanced(
      CommandXboxController controller,
      BooleanSupplier rumbleSupplier,
      double rumbleTime,
      double rumbleStrength) {
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
    return Timer.getFPGATimestamp() - startTime >= rumbleTime || rumbleSupplier.getAsBoolean();
  }
}
