// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance = null;

  private GamePiece desiredGamePiece;

  private LogicalGridLocation desiredLogicalGrid;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  private RobotState() {
    setDesiredGamePiece(GamePiece.CUBE);
    setDesiredLogicalGrid(LogicalGridLocation.LOGICAL_BAY_1);
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
}