package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.NODE_DISTANCES;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.stinger.StingerSetExtension;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;

/** MechanismPositions */
// We'll need: low, mid, high, portal pickup, ground pickup
public class MechanismPositions {

  // TODO: find the proper stow and pickup positions based off robot measurements
  static final double stowHeight = 0;
  static final double stowExtension = 0;
  static final double groundPickupHeight = 1;
  static final double groundPickupExtension = 5;
  static final double substationPickupHeight = 45;
  static final double substationPickupExtension = 20;

  private MechanismPositions() {}

  public static Command scoreLowPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_LOW),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () -> (elevator.getHeightInches() >= Constants.NODE_DISTANCES.HEIGHT_LOW / 2)),
                new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_LOW))));
  }

  public static Command scoreCubeMidPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_MID_CUBE),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () ->
                        (elevator.getHeightInches()
                            >= Constants.NODE_DISTANCES.HEIGHT_MID_CUBE / 2)),
                new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_MID))));
  }

  public static Command scoreConeMidPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_MID_CONE),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () ->
                        (elevator.getHeightInches()
                            >= Constants.NODE_DISTANCES.HEIGHT_MID_CONE / 2)),
                new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_MID))));
  }

  public static Command scoreCubeHighPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () ->
                        (elevator.getHeightInches()
                            >= Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE / 2)),
                new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_HIGH))));
  }

  public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_HIGH_CONE),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () ->
                        (elevator.getHeightInches()
                            >= Constants.NODE_DISTANCES.HEIGHT_HIGH_CONE / 2)),
                new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_HIGH))));
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
        new StingerSetExtension(stinger, 0),
        new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> stinger.getExtensionInches() <= NODE_DISTANCES.EXTENSION_MID / 2),
            new ElevatorSetHeight(elevator, 0)));
  }
}
