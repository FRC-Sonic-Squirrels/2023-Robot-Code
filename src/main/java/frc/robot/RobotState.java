// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance = null;

  private GamePiece desiredGamePiece;

  private LogicalGridLocation desiredLogicalGrid;

  private ScoringRow desiredScoringHeight;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  private RobotState() {
    setDesiredGamePiece(GamePiece.CUBE);
    setDesiredLogicalGrid(LogicalGridLocation.LOGICAL_BAY_1);
    setDesiredScoringHeight(ScoringRow.High);
  }

  public LogicalGridLocation getDesiredLogicalGrid() {
    return desiredLogicalGrid;
  }

  public GamePiece getDesiredGamePiece() {
    return desiredGamePiece;
  }

  public void setDesiredGamePiece(GamePiece gamePiece) {
    Logger.getInstance().recordOutput("RobotState/DesiredGamePiece", gamePiece.name());
    desiredGamePiece = gamePiece;
  }

  public void setDesiredLogicalGrid(LogicalGridLocation gridLocation) {
    Logger.getInstance().recordOutput("RobotState/DesiredLogicalGridLocation", gridLocation.name());
    desiredLogicalGrid = gridLocation;
  }

  public void setDesiredScoringHeight(ScoringRow height) {
    Logger.getInstance()
        .recordOutput("RobotState/DesiredScoringHeight", desiredScoringHeight.name());
    desiredScoringHeight = height;
  }

  public void incrementDesiredBay() {
    var gridOrder = GridPositionHandler.logicalGridOrder;
    var currentIndex = getBayIndex(desiredLogicalGrid);

    int targetIndex;

    if (currentIndex == gridOrder.length - 1) {
      targetIndex = gridOrder.length - 1;
    } else {
      targetIndex = currentIndex + 1;
    }

    setDesiredLogicalGrid(gridOrder[targetIndex]);
  }

  public void decrementDesiredBay() {
    var gridOrder = GridPositionHandler.logicalGridOrder;
    var currentIndex = getBayIndex(desiredLogicalGrid);

    int targetIndex;

    if (currentIndex == 0) {
      targetIndex = 0;
    } else {
      targetIndex = currentIndex - 1;
    }

    setDesiredLogicalGrid(gridOrder[targetIndex]);
  }

  private int getBayIndex(LogicalGridLocation location) {
    var gridOrder = GridPositionHandler.logicalGridOrder;

    for (int i = 0; i < gridOrder.length; i++) {
      if (location == gridOrder[i]) {
        return i;
      }
    }

    return -1;
  }

  public enum GamePiece {
    CONE,
    CUBE;
  }

  public enum ScoringRow {
    Hybrid,
    Mid,
    High
  }

  public Command getScoringCommandBasedOnCurrentRobotState(
      Elevator elevator, Stinger stinger, Intake intake, CommandXboxController driverController) {
    if (desiredGamePiece == GamePiece.CONE) {
      switch (desiredScoringHeight) {
        case Hybrid:
          return MechanismActions.scoreConeHybrid(elevator, stinger, intake, driverController);
        case Mid:
          return MechanismActions.scoreConeMid(elevator, stinger, intake, driverController);
        case High:
          return MechanismActions.scoreConeHigh(elevator, stinger, intake, driverController);
        default:
          return null;
      }
    } else {
      switch (desiredScoringHeight) {
        case Hybrid:
          return MechanismActions.scoreCubeHybrid(elevator, stinger, intake, driverController);
        case Mid:
          return MechanismActions.scoreCubeMid(elevator, stinger, intake, driverController);
        case High:
          return MechanismActions.scoreCubeHigh(elevator, stinger, intake, driverController);
        default:
          return null;
      }
    }
  }
}
