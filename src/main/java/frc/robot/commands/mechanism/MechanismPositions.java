package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.elevator.ElevatorFollowCurve;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.stinger.StingerFollowCurve;
import frc.robot.commands.stinger.StingerSetExtension;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;

/** MechanismPositions */
// We'll need: low, mid, high, portal pickup, ground pickup
public class MechanismPositions {

  // TODO: find the proper stow and pickup positions based off robot measurements
  static final double stowHeight = 6;
  static final double stowExtension = 0;
  static final double groundPickupHeight = 3;
  static final double groundPickupExtension = 11;
  static final double substationPickupHeight = 20;
  static final double substationPickupExtension = 20;
  static final double elevatorAboveBumberHeight = 3;
  private static final TunableNumber elevatorHeightThreshold =
      new TunableNumber("MechPosCommand/elevatorHeightThreshold", 20);
  private static final TunableNumber stingerExtensionThreshold =
      new TunableNumber("MechPosCommand/stingerExtensionThreshold", 5);

  private MechanismPositions() {}

  public static Command scoreLowPosition(
      Elevator elevator, Stinger stinger, Intake intake, GamePiece gamepiece) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_LOW,
            Constants.NODE_DISTANCES.EXTENSION_LOW,
            false,
            new InstantCommand()),
        new ConditionalCommand(
            new IntakeScoreCone(intake).withTimeout(0.2),
            new IntakeScoreCube(intake).withTimeout(0.2),
            () -> gamepiece == GamePiece.CONE),
        safeZero(elevator, stinger));
  }

  public static Command scoreCubeMidPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_MID_CUBE,
            Constants.NODE_DISTANCES.EXTENSION_MID_CUBE,
            true,
            new IntakeGrabCube(intake).withTimeout(0.1)),
        new IntakeScoreCube(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  public static Command scoreConeMidPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_MID_CONE,
            Constants.NODE_DISTANCES.EXTENSION_MID_CONE,
            true,
            new IntakeGrabCone(intake).withTimeout(0.2)),
        new IntakeScoreCone(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  public static Command scoreCubeHighPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE,
            Constants.NODE_DISTANCES.EXTENSION_HIGH_CUBE,
            true,
            new IntakeGrabCube(intake).withTimeout(0.1)),
        new IntakeScoreCube(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_HIGH_CONE,
            Constants.NODE_DISTANCES.EXTENSION_HIGH_CONE,
            true,
            new IntakeGrabCone(intake).withTimeout(0.2)),
        new IntakeScoreCone(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  public static Command groundPickupPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, groundPickupExtension),
        new ElevatorSetHeight(elevator, groundPickupHeight));
  }

  public static Command substationPickupPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        substationPickupHeight,
        substationPickupExtension,
        false,
        new InstantCommand());
  }

  public static Command stowPosition(Elevator elevator, Stinger stinger) {
    return new ConditionalCommand(
        new SequentialCommandGroup(
            new StingerSetExtension(stinger, stowExtension),
            new ElevatorSetHeight(elevator, stowHeight)),
        new SequentialCommandGroup(
            new ElevatorSetHeight(elevator, stowHeight),
            new StingerSetExtension(stinger, stowExtension)),
        () -> (elevator.getHeightInches() > stowHeight));
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
      Elevator elevator,
      Stinger stinger,
      double heightInches,
      double extensionInches,
      boolean suck,
      Command suckCommand) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, 0),
        new ElevatorSetHeight(elevator, stowHeight),
        new ElevatorSetHeight(elevator, heightInches),
        new ParallelCommandGroup(
            new StingerSetExtension(stinger, extensionInches),
            new ConditionalCommand(suckCommand, new InstantCommand(), () -> suck)));
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

  public static Command testScoreConeHigh(Elevator elevator, Stinger stinger, Intake intake) {

    return new SequentialCommandGroup(
        new ElevatorSetHeight(elevator, Constants.NODE_DISTANCES.HEIGHT_HIGH_CONE),
        // --
        new ParallelCommandGroup(
            new IntakeGrabCone(intake, 0.8).withTimeout(0.1),
            new StingerSetExtension(stinger, Constants.NODE_DISTANCES.EXTENSION_HIGH_CONE)),
        // --
        new IntakeScoreCone(intake).withTimeout(0.25));
  }

  public static Command safeZero(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, 0),
        new ElevatorSetHeight(elevator, 0));
  }
}
