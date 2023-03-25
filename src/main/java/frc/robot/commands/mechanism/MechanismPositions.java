package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj.Timer;
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
      new TunableNumber("MechPosCommand/elevatorHeightThreshold", 20);
  private static final TunableNumber stingerExtensionThreshold =
      new TunableNumber("MechPosCommand/stingerExtensionThreshold", 20);

  public static final TunableNumber yeetElevatorHeight =
      new TunableNumber("yeet/elevatorHeight", 20);
  public static final TunableNumber yeetStingerExtension =
      new TunableNumber("yeet/stingerExtension", 25);
  private static final TunableNumber yeetStingerThreshold =
      new TunableNumber("yeet/stingerThreshold", 15);

  public static Timer movementTimer = new Timer();

  private MechanismPositions() {}

  // TODO: figure out where to log this
  // Logger.getInstance().recordOutput("MechPosCommand/movementTimer", movementTimer.get());

  // -------- SCORE LOW -------------
  public static Command scoreLow(
      Elevator elevator,
      Stinger stinger,
      Intake intake,
      GamePiece gamepiece,
      CommandXboxController rumbleController) {
    return new SequentialCommandGroup(
        lowPosition(elevator, stinger),
        rumbleButtonConfirmation(rumbleController),
        new ConditionalCommand(
            new IntakeScoreCone(intake).withTimeout(0.2),
            new IntakeScoreCube(intake, 1.0).withTimeout(0.3),
            () -> gamepiece == GamePiece.CONE),
        safeZero(elevator, stinger));
  }

  public static Command lowPosition(Elevator elevator, Stinger stinger) {
    return goToPositionSimple(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_LOW,
        Constants.NODE_DISTANCES.EXTENSION_LOW);
  }

  // --------SCORE LOW -------------

  // --------CUBE MID -------------

  public static Command scoreCubeMid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreCubeMidLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreCubeMid(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreCubeMidLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreCubeMidLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {

    return new SequentialCommandGroup(
        cubeMidPosition(elevator, stinger, intake),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCube(intake).withTimeout(0.2),
        safeZero(elevator, stinger));
  }

  public static Command cubeMidPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionWithSuck(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_MID_CUBE,
        Constants.NODE_DISTANCES.EXTENSION_MID_CUBE,
        () -> intakeGrabPiece(intake, GamePiece.CUBE, 0.25));
  }

  // --------CUBE MID -------------

  // --------CONE MID -------------

  public static Command scoreConeMid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreConeMidLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreConeMid(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreConeMidLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreConeMidLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {
    return new SequentialCommandGroup(
        coneMidPosition(elevator, stinger, intake),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCone(intake).withTimeout(0.2),
        // --
        safeZero(elevator, stinger));
  }

  public static Command coneMidPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionWithSuck(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_MID_CONE,
        Constants.NODE_DISTANCES.EXTENSION_MID_CONE,
        () -> intakeGrabPiece(intake, GamePiece.CONE, 0.25));
  }

  // --------CONE MID -------------

  // --------CUBE HIGH -------------

  public static Command scoreCubeHigh(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return cubeHighLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreCubeHigh(Elevator elevator, Stinger stinger, Intake intake) {

    return cubeHighLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command cubeHighLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {

    return new SequentialCommandGroup(
        cubeHighPosition(elevator, stinger, intake),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCube(intake, 0.8).withTimeout(0.2),
        // --
        aggressiveZero(elevator, stinger));
  }

  public static Command cubeHighPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionParallelWithSuck(
        elevator,
        stinger,
        Constants.NODE_DISTANCES.HEIGHT_HIGH_CUBE,
        Constants.NODE_DISTANCES.EXTENSION_HIGH_CUBE,
        () -> intakeGrabPieceNoTimeout(intake, GamePiece.CUBE, 0.25));
  }

  // --------CUBE HIGH -------------

  // --------CONE HIGH -------------

  public static Command scoreConeHigh(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController rumbleController) {

    return scoreConeHighLogic(
        elevator, stinger, intake, () -> rumbleButtonConfirmation(rumbleController));
  }

  public static Command scoreConeHigh(Elevator elevator, Stinger stinger, Intake intake) {

    return scoreConeHighLogic(elevator, stinger, intake, () -> new InstantCommand());
  }

  private static Command scoreConeHighLogic(
      Elevator elevator, Stinger stinger, Intake intake, Supplier<Command> confirmationCommand) {

    return new SequentialCommandGroup(
        coneHighPosition(elevator, stinger, intake),
        // --
        confirmationCommand.get(),
        // --
        new IntakeScoreCone(intake).withTimeout(0.2),
        // --
        aggressiveZero(elevator, stinger)
            .deadlineWith(new IntakeScoreCone(intake).withTimeout(0.5)));
  }

  public static Command coneHighPosition(Elevator elevator, Stinger stinger, Intake intake) {
    return goToPositionParallelWithSuck(
        elevator,
        stinger,
        NODE_DISTANCES.HEIGHT_HIGH_CONE,
        NODE_DISTANCES.EXTENSION_HIGH_CONE,
        () -> intakeGrabPieceNoTimeout(intake, GamePiece.CONE, 0.8));
  }

  public static Command yeetCube(Elevator elevator, Stinger stinger, Intake intake) {
    return new ParallelCommandGroup(
        new ElevatorSetHeight(elevator, yeetElevatorHeight.get()),
        new StingerSetExtension(stinger, yeetStingerExtension.get()),
        new WaitUntilCommand(() -> (stinger.getExtensionInches() >= yeetStingerThreshold.get()))
            .andThen(new IntakeScoreCube(intake, 1).withTimeout(0.4)));
  }

  // public static Command scoreConeHighPosition(Elevator elevator, Stinger stinger, Intake intake)
  // {
  //   return new SequentialCommandGroup(
  //     new ParallelCommandGroup(
  //             new ElevatorSetHeight(elevator, NODE_DISTANCES.HEIGHT_HIGH_CONE),
  //             // --
  //             new SequentialCommandGroup(
  //                 Commands.waitUntil(
  //                     () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
  //                 // --

  //                 new StingerSetExtension(stinger, NODE_DISTANCES.EXTENSION_HIGH_CONE)
  //                 // --
  //                 ))
  //         .raceWith(suckCommand.get())
  // }

  // --------CONE HIGH -------------

  private static Command WaitUntilCommand(boolean b) {
    return null;
  }

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
        new ConditionalCommand(
            new ElevatorSetHeight(elevator, 4),
            new InstantCommand(),
            () -> (elevator.getHeightInches() <= elevatorAboveBumberHeight)),
        new StingerSetExtension(stinger, groundPickupExtension),
        new ElevatorSetHeight(elevator, groundPickupHeight));
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

    return new SequentialCommandGroup(
        new InstantCommand(() -> movementTimer.reset()),
        new InstantCommand(() -> movementTimer.start()),
        new ConditionalCommand(
                new ParallelCommandGroup(
                    new StingerSetExtension(stinger, extensionInches),
                    new SequentialCommandGroup(
                        Commands.waitUntil(
                            () ->
                                (stinger.getExtensionInches() <= stingerExtensionThreshold.get())),
                        new ElevatorSetHeight(elevator, heightInches))),
                new ParallelCommandGroup(
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
                        new StingerSetExtension(stinger, extensionInches))),
                () -> elevator.getHeightInches() >= heightInches)
            .raceWith(suckCommand.get()),
        new InstantCommand(() -> movementTimer.stop()));
  }

  public static Command goToPositionParallel(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {

    return new ConditionalCommand(
        new ParallelCommandGroup(
            new StingerSetExtension(stinger, extensionInches),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () -> (stinger.getExtensionInches() <= stingerExtensionThreshold.get())),
                new ElevatorSetHeight(elevator, heightInches))),
        new ParallelCommandGroup(
            new ElevatorSetHeight(elevator, heightInches),
            new SequentialCommandGroup(
                Commands.waitUntil(
                    () -> (elevator.getHeightInches() >= elevatorHeightThreshold.get())),
                new StingerSetExtension(stinger, extensionInches))),
        () -> elevator.getHeightInches() >= heightInches);
  }

  public static Command goToPositionWithSuck(
      Elevator elevator,
      Stinger stinger,
      double heightInches,
      double extensionInches,
      Supplier<Command> suckCommandSupplier) {

    return new SequentialCommandGroup(
        new InstantCommand(() -> movementTimer.reset()),
        new InstantCommand(() -> movementTimer.start()),
        new ConditionalCommand(
            new SequentialCommandGroup(
                avoidBumper(elevator, stinger),
                // new StingerSetExtension(stinger, 0),
                // new ElevatorSetHeight(elevator, stowHeight),
                new ParallelCommandGroup(
                    new StingerSetExtension(stinger, extensionInches), suckCommandSupplier.get()),
                new ParallelCommandGroup(
                    new ElevatorSetHeight(elevator, heightInches), suckCommandSupplier.get())),
            new SequentialCommandGroup(
                avoidBumper(elevator, stinger),
                // new StingerSetExtension(stinger, 0),
                // new ElevatorSetHeight(elevator, stowHeight),
                new ParallelCommandGroup(
                    new ElevatorSetHeight(elevator, heightInches), suckCommandSupplier.get()),
                new ParallelCommandGroup(
                    new StingerSetExtension(stinger, extensionInches), suckCommandSupplier.get())),
            () -> (elevator.getHeightInches() >= heightInches)),
        new InstantCommand(() -> movementTimer.stop()));
  }

  public static Command goToPositionSimple(
      Elevator elevator, Stinger stinger, double heightInches, double extensionInches) {

    return new SequentialCommandGroup(
        new InstantCommand(() -> movementTimer.reset()),
        new InstantCommand(() -> movementTimer.start()),
        new ConditionalCommand(
            new SequentialCommandGroup(
                // FIXME: not having aviod bumper seems scary
                // avoidBumper(elevator, stinger),
                new ElevatorSetHeight(elevator, heightInches),
                new StingerSetExtension(stinger, extensionInches)),
            new SequentialCommandGroup(
                // avoidBumper(elevator, stinger),
                new StingerSetExtension(stinger, extensionInches),
                new ElevatorSetHeight(elevator, heightInches)),
            // FIXME: i think this is wrong gpk had weird behavior
            () -> (elevator.getHeightInches() <= heightInches)),
        new InstantCommand(() -> movementTimer.stop()));
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
      Elevator elevator,
      Stinger stinger,
      double heightInches,
      double extensionInches,
      Supplier<Command> suckCommand) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> movementTimer.reset()),
        new InstantCommand(() -> movementTimer.start()),
        new ConditionalCommand(
                new ElevatorFollowCurve(elevator, stinger, extensionInches),
                new StingerFollowCurve(elevator, stinger, heightInches),
                () -> elevator.getHeightInches() >= heightInches)
            .raceWith(suckCommand.get()),
        new InstantCommand(() -> movementTimer.stop()));
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
        goToPositionSimple(elevator, stinger, 8, 0),
        new ElevatorSetHeight(elevator, 0.0, () -> 15, () -> 0.5));
  }

  public static Command aggressiveZero(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger),
        goToPositionParallel(elevator, stinger, 8, 0),
        new ElevatorSetHeight(elevator, 0.0, () -> 15, () -> 0.5));
  }

  public static Command safeStowPosition(Elevator elevator, Stinger stinger) {
    return new SequentialCommandGroup(
        avoidBumper(elevator, stinger), goToPositionSimple(elevator, stinger, 8.0, 0.0));
  }

  public static Command rumbleCanMove(CommandXboxController controller) {
    return new ControllerRumbleInterval(controller, 3, 0.2, 0.25, 0.8);
  }

  public static Command rumbleButtonConfirmation(CommandXboxController controller) {
    return new ControllerRumbleUntilButtonPress(
        controller, () -> controller.rightTrigger(0.4).getAsBoolean(), 0.7);
  }
}
