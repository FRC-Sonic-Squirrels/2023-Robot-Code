package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleUntilButtonPress;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.ScoringRow;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;

public class MechanismActions {

  // ------------ HYBRID -----------------------
  public static Command scoreConeHybrid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CONE, ScoringRow.Hybrid, 0.3, elevator, stinger, intake, driverController);
  }

  public static Command scoreCubeHybrid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CUBE, ScoringRow.Hybrid, 0.3, elevator, stinger, intake, driverController);
  }

  // ---------------------- MID ----------------------------
  public static Command scoreConeMid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CONE, ScoringRow.Mid, 0.3, elevator, stinger, intake, driverController);
  }

  public static Command scoreCubeMid(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CUBE, ScoringRow.Mid, 0.5, elevator, stinger, intake, driverController);
  }

  // --------------------------- HIGH -----------------------------

  public static Command scoreConeHigh(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CONE, ScoringRow.High, 0.3, elevator, stinger, intake, driverController);
  }

  public static Command scoreCubeHigh(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    return scoreGenericLocation(
        GamePiece.CUBE, ScoringRow.High, 0.4, elevator, stinger, intake, driverController);
  }

  private static Command scoreGenericLocation(
      GamePiece gamePiece,
      ScoringRow scoringRow,
      double scoringTime,
      Elevator elevator,
      Stinger stinger,
      Intake intake,
      CommandXboxController driverController) {
    Command positionCommand = getPositionCommand(gamePiece, scoringRow, elevator, stinger);

    return new SequentialCommandGroup(
        positionCommand.deadlineWith(intakeSuckWhileScoring(gamePiece, intake)),
        // --
        rumbleDriverControllerForConformation(driverController)
            .deadlineWith(intakeSuckWhileScoring(gamePiece, intake)),
        // --
        scoreForGamePieceCommand(gamePiece, intake).withTimeout(scoringTime),
        // --
        MechanismPositions.aggressiveZero(elevator, stinger)
            .deadlineWith(scoreForGamePieceCommand(gamePiece, intake).withTimeout(0.5)),
        MechanismPositions.stowPosition(elevator, stinger));
  }

  private static Command intakeSuckWhileScoringCone(Intake intake) {
    return new IntakeGrabCone(intake, 0.8);
  }

  private static Command intakeSuckWhileScoringCube(Intake intake) {
    return new IntakeGrabCube(intake, 0.25);
  }

  private static Command intakeSuckWhileScoring(GamePiece gamePiece, Intake intake) {
    return gamePiece == GamePiece.CONE
        ? intakeSuckWhileScoringCone(intake)
        : intakeSuckWhileScoringCube(intake);
  }

  private static Command scoreForGamePieceCommand(GamePiece gamePiece, Intake intake) {
    return gamePiece == GamePiece.CONE ? new IntakeScoreCone(intake) : new IntakeScoreCube(intake);
  }

  public static Command grabForGamePiceCommand(GamePiece gamePiece, Intake intake) {
    return gamePiece == GamePiece.CONE ? new IntakeGrabCone(intake) : new IntakeGrabCube(intake);
  }

  private static Command getPositionCommand(
      GamePiece gamePiece, ScoringRow scoringRow, Elevator elevator, Stinger stinger) {
    if (gamePiece == GamePiece.CONE) {
      switch (scoringRow) {
        case Hybrid:
          return MechanismPositions.lowPosition(elevator, stinger);

        case Mid:
          return MechanismPositions.coneMidPosition(elevator, stinger);

        case High:
          return MechanismPositions.coneHighPosition(elevator, stinger);

        default:
          return null;
      }
    } else {
      switch (scoringRow) {
        case Hybrid:
          return MechanismPositions.lowPosition(elevator, stinger);

        case Mid:
          return MechanismPositions.cubeMidPosition(elevator, stinger);

        case High:
          return MechanismPositions.cubeHighPosition(elevator, stinger);

        default:
          return null;
      }
    }
  }

  private static Command rumbleDriverControllerForConformation(CommandXboxController controller) {
    return new ControllerRumbleUntilButtonPress(
        controller, () -> controller.a().getAsBoolean(), 0.7);
  }
}
