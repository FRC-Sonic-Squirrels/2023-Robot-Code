package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;

/** MechanismPositions */
// We'll need: low, mid, high, portal pickup, ground pickup
public class MechanismPositions {

  private MechanismPositions() {}

  public static Command scoreLowPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setHeightInches(24)),
        new InstantCommand(() -> stinger.setExtensionInches(10)));
  }
}
