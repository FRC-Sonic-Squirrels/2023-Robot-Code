package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorFollowCurve;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.stinger.StingerFollowCurve;
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
  static final double substationPickupHeight = 20;
  static final double substationPickupExtension = 20;
  static final double elevatorAboveBumberHeight = 5;
  private static final TunableNumber elevatorHeightThreshold =
      new TunableNumber("MechPosCommand/elevatorHeightThreshold", 20);
  private static final TunableNumber stingerExtensionThreshold =
      new TunableNumber("MechPosCommand/stingerExtensionThreshold", 5);

  private MechanismPositions() {}

  public static Command scoreLowPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_LOW,
        Constants.NODE_DISTANCES.EXTENSION_LOW);
  }

  public static Command scoreCubeMidPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_MID_CUBE,
        Constants.NODE_DISTANCES.EXTENSION_MID);
  }

  public static Command scoreConeMidPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_MID_CONE,
        Constants.NODE_DISTANCES.EXTENSION_MID);
  }

  public static Command scoreCubeHighPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE,
        Constants.NODE_DISTANCES.EXTENSION_HIGH);
  }

  public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_HIGH_CONE,
        Constants.NODE_DISTANCES.EXTENSION_HIGH);
  }

  public static Command groundPickupPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, groundPickupExtension),
        new ElevatorSetHeight(elevator, groundPickupHeight));
  }

  public static Command substationPickupPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(elevator, stinger, substationPickupHeight, substationPickupExtension);
  }

  public static Command stowPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, stowExtension),
        new ElevatorSetHeight(elevator, stowHeight));
  }

  public static Command goToPosition(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {
    if (elevator.getHeightInches() >= heightInches) {
      return new ParallelCommandGroup(
          new StingerSetExtension(stinger, extensionInches),
          new SequentialCommandGroup(
              Commands.waitUntil(
                  () -> (stinger.getExtensionInches() <= stingerExtensionThreshold.get())),
              new ElevatorSetHeight(elevator, heightInches)));
    } else {
      return new ParallelCommandGroup(
          new ElevatorSetHeight(elevator, heightInches),
          new SequentialCommandGroup(
              Commands.waitUntil(
                  () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
              new StingerSetExtension(stinger, extensionInches)));
    }
  }

  public static Command goToPositionSimple(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, 0),
        new ElevatorSetHeight(elevator, elevatorAboveBumberHeight),
        new ElevatorSetHeight(elevator, heightInches),
        new StingerSetExtension(stinger, extensionInches));
  }

  public static Command avoidBumper(Elevator elevator, Stinger stinger) {
    return new ConditionalCommand(
        new ElevatorSetHeight(elevator, elevatorAboveBumberHeight),
        new InstantCommand(),
        (() -> (elevator.getHeightInches() < elevatorAboveBumberHeight)));
  }

  public static Command goToPositionCurve(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {
    if (elevator.getHeightInches() >= heightInches) {
      return new ElevatorFollowCurve(elevator, stinger, extensionInches);
    } else {
      return new StingerFollowCurve(elevator, stinger, heightInches);
    }
  }
}