package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleInterval;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleUntilButtonPress;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.NODE_DISTANCES;
import frc.robot.RobotState;
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
import java.util.function.Supplier;

/** MechanismPositions */
// We'll need: low, mid, high, portal pickup, ground pickup
public class MechanismPositions {

  // TODO: find the proper stow and pickup positions based off robot measurements
  static final double stowHeight = Constants.NODE_DISTANCES.STOW_HEIGHT;
  static final double stowExtension = Constants.NODE_DISTANCES.STOW_EXTENSION;
  static final double groundPickupHeight = 3;
  static final double groundPickupExtension = 11;
  static final double substationPickupHeight = 20;
  static final double substationPickupExtension = 20;
  static final double elevatorAboveBumberHeight = 3;
  private static final TunableNumber elevatorHeightThreshold =
      new TunableNumber("MechPosCommand/elevatorHeightThreshold", 40);
  private static final TunableNumber stingerExtensionThreshold =
      new TunableNumber("MechPosCommand/stingerExtensionThreshold", 5);

  private MechanismPositions() {}

  // -------- SCORE LOW -------------
  public static Command scoreLowPosition(
      Elevator elevator,
      Stinger stinger,
      Intake intake,
      GamePiece gamepiece,
      CommandXboxController rumbleController) {
    return new SequentialCommandGroup(
        goToPositionSimple(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_LOW,
            Constants.NODE_DISTANCES.EXTENSION_LOW),
        rumbleButtonConfirmation(rumbleController),
        new ConditionalCommand(
            new IntakeScoreCone(intake).withTimeout(0.2),
            new IntakeScoreCube(intake).withTimeout(0.2),
            () -> gamepiece == GamePiece.CONE),
        safeZero(elevator, stinger));
  }

  // --------SCORE LOW -------------

  // --------CUBE MID -------------

  public static Command scoreCubeMidPosition(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreCubeMidLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreCubeMidPosition(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreCubeMidLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreCubeMidLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {

    return new SequentialCommandGroup(
        goToPositionWithSuck(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_MID_CUBE,
            Constants.NODE_DISTANCES.EXTENSION_MID_CUBE,
            () -> intakeGrabPiece(intake, GamePiece.CUBE, 0.25)),

        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCube(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  // --------CUBE MID -------------

  // --------CONE MID -------------

  public static Command scoreConeMidPosition(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreConeMidLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreConeMidPosition(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreConeMidLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreConeMidLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {
    return new SequentialCommandGroup(
        goToPositionWithSuck(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_MID_CONE,
            Constants.NODE_DISTANCES.EXTENSION_MID_CONE,
            () -> intakeGrabPiece(intake, GamePiece.CONE, 0.25)),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCone(intake).withTimeout(0.2),
        // --
        safeZero(elevator, stinger));
  }

  // --------CONE MID -------------

  // --------CUBE HIGH -------------

  public static Command scoreCubeHighPosition(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreCubeHighLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreCubeHighPosition(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreCubeHighLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreCubeHighLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {

    return new SequentialCommandGroup(
        goToPositionParallelWithSuck(
            elevator,
            stinger,
            Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE,
            Constants.NODE_DISTANCES.EXTENSION_HIGH_CUBE,
            () -> intakeGrabPiece(intake, GamePiece.CUBE, 0.25)),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCube(intake).withTimeout(0.2),
        // --
        safeZero(elevator, stinger));
  }

  // --------CUBE HIGH -------------

  // --------CONE HIGH -------------

  public static Command scoreConeHighPosition(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreConeHighLogic(
        elevator,
        stinger,
        intake,
        () -> rumbleButtonConfirmation(rumbleController),
        () -> intakeGrabPieceNoTimeout(intake, GamePiece.CONE, 0.25));
  }

  public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreConeHighLogic(
        elevator,
        stinger,
        intake,
        () -> new InstantCommand(),
        () -> intakeGrabPieceNoTimeout(intake, GamePiece.CONE, 0.25));
  }

  private static Command scoreConeHighLogic(
      Elevator elevator,
      Stinger stinger,
      Intake intake,
      Supplier<Command> confirmationCommand,
      Supplier<Command> suckCommand) {

    return new SequentialCommandGroup(
        new ParallelCommandGroup(
                new ElevatorSetHeight(elevator, NODE_DISTANCES.HEIGHT_HIGH_CONE),
                // --
                new SequentialCommandGroup(
                    Commands.waitUntil(
                        () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
                    // --

                    new StingerSetExtension(stinger, NODE_DISTANCES.EXTENSION_HIGH_CONE)
                    // --
                    ))
            .raceWith(suckCommand.get()),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCone(intake).withTimeout(0.2),
        // --
        safeZero(elevator, stinger));
  }

  // --------CONE HIGH -------------

  public static Command intakeGrabPiece(Intake intake) {
    return intakeGrabPiece(intake, RobotState.getInstance().getDesiredGamePiece());
  }

  public static Command intakeGrabPiece(Intake intake, GamePiece gamepiece) {
    return intakeGrabPiece(intake, gamepiece, 1);
  }

  public static Command intakeGrabPiece(Intake intake, GamePiece gamepiece, double speed) {

    return new ConditionalCommand(
        new IntakeGrabCone(intake, speed).withTimeout(0.25),
        new IntakeGrabCube(intake, speed).withTimeout(0.25),
        () -> (gamepiece == GamePiece.CONE));
  }

  public static Command intakeGrabPieceNoTimeout(Intake intake, GamePiece gamepiece, double speed) {

    return new ConditionalCommand(
        new IntakeGrabCone(intake, speed),
        new IntakeGrabCube(intake, speed),
        () -> (gamepiece == GamePiece.CONE));
  }

  public static Command intakeGrabPiece(
      Intake intake, GamePiece gamepiece, double speedRPM, double timeout) {
    return intakeGrabPiece(intake, gamepiece, speedRPM).withTimeout(timeout);
  }

  public static Command groundPickupPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new ElevatorSetHeight(elevator, groundPickupHeight),
        new StingerSetExtension(stinger, groundPickupExtension));
  }

  // public static Command substationPickupPosition(Elevator elevator, Stinger stinger) {
  //   return goToPositionSimple(
  //       elevator,
  //       stinger,
  //       substationPickupHeight,
  //       substationPickupExtension,
  //       false,
  //       new InstantCommand());
  // }

  public static Command substationPickupPositionCone(
      Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionSimple(elevator, stinger, 45.7, 0);
    // .alongWith(new IntakeGrabCone(intake));
  }

  public static Command substationPickupPositionCube(
      Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionSimple(elevator, stinger, 42, 0);
    // .alongWith(new IntakeGrabCone(intake));
  }

  public static Command stowPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(elevator, stinger, stowHeight, stowExtension);
  }

  public static Command goToPositionParallelWithSuck(
      Elevator elevator,
      Stinger stinger,
      double heightInches,
      double extensionInches,
      Supplier<Command> suckCommand) {
    if (elevator.getHeightInches() >= heightInches) {
      return new ParallelCommandGroup(
          new StingerSetExtension(stinger, extensionInches),
          new SequentialCommandGroup(
              Commands.waitUntil(
                  () -> (stinger.getExtensionInches() <= stingerExtensionThreshold.get())),
              new ElevatorSetHeight(elevator, heightInches)));
    } else {
      return new ParallelCommandGroup(
              // new ElevatorSetHeight(elevator, heightInches),
              // new SequentialCommandGroup(
              //     Commands.waitUntil(
              //         () -> (elevator.getHeightInches() >= elevatorAboveBumberHeight)),
              //     new StingerSetExtension(stinger, 4.5)
              //         .until(() -> elevator.getHeightInches() >= 29.2),
              //     // --
              //     Commands.waitUntil(() -> (elevator.getHeightInches() >= 29.2)),
              //     new StingerSetExtension(stinger, 10)
              //         .until(() -> elevator.getHeightInches() >= 40),
              //     // --
              //     Commands.waitUntil(
              //         () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
              //     new StingerSetExtension(stinger, extensionInches))
              new ElevatorSetHeight(elevator, heightInches),
              new SequentialCommandGroup(
                  Commands.waitUntil(
                      () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
                  new StingerSetExtension(stinger, extensionInches)))
          .alongWith(suckCommand.get());
    }
  }

  public static Command goToPositionWithSuck(
      Elevator elevator,
      Stinger stinger,
      double heightInches,
      double extensionInches,
      Supplier<Command> suckCommandSupplier) {
    if (elevator.getHeightInches() >= heightInches) {
      return new SequentialCommandGroup(
          avoidBumper(elevator, stinger),
          // new StingerSetExtension(stinger, 0),
          // new ElevatorSetHeight(elevator, stowHeight),
          new ParallelCommandGroup(
              new StingerSetExtension(stinger, extensionInches), suckCommandSupplier.get()),
          new ParallelCommandGroup(
              new ElevatorSetHeight(elevator, heightInches), suckCommandSupplier.get()));
    } else {
      return new SequentialCommandGroup(
          avoidBumper(elevator, stinger),
          // new StingerSetExtension(stinger, 0),
          // new ElevatorSetHeight(elevator, stowHeight),
          new ParallelCommandGroup(
              new ElevatorSetHeight(elevator, heightInches), suckCommandSupplier.get()),
          new ParallelCommandGroup(
              new StingerSetExtension(stinger, extensionInches), suckCommandSupplier.get()));
    }
  }

  public static Command goToPositionSimple(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {
    if (elevator.getHeightInches() <= heightInches) {
      return new SequentialCommandGroup(
          avoidBumper(elevator, stinger),
          new ElevatorSetHeight(elevator, heightInches),
          new StingerSetExtension(stinger, extensionInches));
    } else {
      return new SequentialCommandGroup(
          avoidBumper(elevator, stinger),
          new StingerSetExtension(stinger, extensionInches),
          new ElevatorSetHeight(elevator, heightInches));
    }
  }

  public static Command avoidBumper(Elevator elevator, Stinger stinger) {
    return new ConditionalCommand(
        new ElevatorSetHeight(elevator, elevatorAboveBumberHeight),
        new InstantCommand(),
        (() ->
            (elevator.getHeightInches() < elevatorAboveBumberHeight)
                && stinger.getExtensionInches() >= 2));
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

  public static Command safeZero(
      Elevator elevator, Stinger stinger, CommandXboxController controllerToRumble) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new ParallelCommandGroup(
            new StingerSetExtension(stinger, 0),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> stinger.getExtensionInches() < 5),
                rumbleCanMove(controllerToRumble))),
        new ElevatorSetHeight(elevator, 0));
  }

  public static Command safeZero(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        new StingerSetExtension(stinger, 0.0),
        new ElevatorSetHeight(elevator, 8),
        new ElevatorSetHeight(elevator, 0.0, () -> 15, () -> 0.5));
  }

  public static Command rumbleCanMove(CommandXboxController controller) {
    return new ControllerRumbleInterval(controller, 3, 0.2, 0.25, 0.8);
  }

  public static Command rumbleButtonConfirmation(CommandXboxController controller) {
    return new ControllerRumbleUntilButtonPress(
        controller, () -> controller.rightTrigger(0.4).getAsBoolean(), 0.7);
  }
}
