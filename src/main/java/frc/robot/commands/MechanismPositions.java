package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.stinger.StingerSetExtension;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;

/** MechanismPositions */
// We'll need: low, mid, high, portal pickup, ground pickup
public class MechanismPositions {

  static final double lowHeight = 10;
  static final double midHeight = 20;
  static final double highHeight = 20;
  static final double lowExtension = 20;
  static final double midExtension = 20;
  static final double highExtension = 20;
  static final double stowHeight = 10;
  static final double stowExtension = 0;
  static final double groundPickupHeight = 1;
  static final double groundPickupExtension = 1;
  static final double substationPickupHeight = 20;
  static final double substationPickupExtension = 20;

  private MechanismPositions() {}

  public static Command scoreLowPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, lowHeight),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (elevator.getHeightInches() >= lowHeight / 2)),
                new StingerSetExtension(stinger, lowExtension))));
  }

  public static Command scoreCubeMidPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, midHeight),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (elevator.getHeightInches() >= midHeight / 2)),
                new StingerSetExtension(stinger, midExtension))));
  }

  public static Command scoreConeMidPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, midHeight),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (elevator.getHeightInches() >= midHeight / 2)),
                new StingerSetExtension(stinger, midExtension))));
  }

  public static Command scoreCubeHighPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, highHeight),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (elevator.getHeightInches() >= highHeight / 2)),
                new StingerSetExtension(stinger, highExtension))));
  }

  public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, highHeight),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (elevator.getHeightInches() >= highHeight / 2)),
                new StingerSetExtension(stinger, highExtension))));
  }

  public static Command groundPickupPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new StingerSetExtension(stinger, groundPickupExtension),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> (stinger.getExtensionInches() >= groundPickupExtension)),
                new ElevatorSetHeight(elevator, groundPickupHeight))));
  }

  public static Command substationPickupPosition(Elevator elevator) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, substationPickupHeight)
            // new SequentialCommandGroup(
            //     Commands.waitUntil(() -> (elevator.getHeightInches() >= substationPickupHeight)),
            //     new StingerSetExtension(stinger, substationPickupExtension))
            ));
  }

  public static Command stowPosition(Elevator elevator, Stinger stinger) {
    return new ParallelCommandGroup(
        new ElevatorSetHeight(elevator, 10),
        new SequentialCommandGroup(
            // height must be >= 10 before moving stinger,
            // if any lower the stinger will collide with swerve module
            Commands.waitUntil(() -> (elevator.getHeightInches() >= 10)),
            new StingerSetExtension(stinger, 0)));
  }
}
