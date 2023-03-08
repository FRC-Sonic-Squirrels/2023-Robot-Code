// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import frc.robot.RobotState;

import java.util.Arrays;

/** Add your docs here. */
public class StreamdeckReal implements StreamdeckIO {

  public String[] TargetingButtons = {"0_1", "0_2", "0_3", "0_4", "0_5", "0_6", "0_7", "0_8", "0_9",
          "1_1", "1_2",
          "1_3", "1_4", "1_5", "1_6", "1_7", "1_8", "1_9", "2_1", "2_2", "2_3", "2_4", "2_5", "2_6", "2_7", "2_8", "2_9"};
  public String[] OverrideButtons = {"0_0", "1_0", "2_0"};

  public String[] AllButtons = {"0_0", "0_1", "0_2", "0_3", "0_4", "0_5", "0_6", "0_7", "0_8", "0_9",
          "1_0", "1_1", "1_2",
          "1_3", "1_4", "1_5", "1_6", "1_7", "1_8", "1_9", "2_0", "2_1", "2_2", "2_3", "2_4", "2_5", "2_6", "2_7", "2_8", "2_9"};

  public StreamdeckReal() {
    init();
  }

  @Override
  public Trigger getButton(String buttonNumber) {
    return new Trigger(() -> SmartDashboard.getBoolean("/streamdeck/" + buttonNumber, false));
  }

  @Override
  public boolean getPressed(String buttonNumber) {
    return new Trigger(() -> SmartDashboard.getBoolean("/streamdeck/" + buttonNumber, false)).getAsBoolean();
  }
  @Override
  public void setButton(String buttonNumber, boolean value) {
    SmartDashboard.putBoolean("/streamdeck/" + buttonNumber, value);
  }
@Override
  public void setDesiredFromButton(String buttonNumber){
  // if the button is not a targeting button, return
  if (!Arrays.asList(TargetingButtons).contains(buttonNumber)) {
    return;
  }
  int desiredGrid = Integer.parseInt(buttonNumber.substring(2)) - 1;
  LogicalGridLocation desiredLocation = LogicalGridLocation.values()[desiredGrid];
  RobotState.getInstance().setDesiredLogicalGrid(desiredLocation);
  Commands.print("Desired: " + desiredLocation).schedule();
  }
@Override
  public LogicalGridLocation getDesiredFromButton(String buttonNumber){
    // if the button is not a targeting button, return
    if (!Arrays.asList(TargetingButtons).contains(buttonNumber)) {
      return null;
    }
    int desiredGrid = Integer.parseInt(buttonNumber.substring(2)) - 1;
    return LogicalGridLocation.values()[desiredGrid];
  }

  @Override
  public boolean[][] getButtonsPressed(boolean[][] ButtonGrid){
    for (int i = 0; i < ButtonGrid.length; i++) {
      for (int j = 0; j < ButtonGrid[i].length; j++) {
        ButtonGrid[i][j] = getPressed(i + "_" + j);
      }
    }
    return ButtonGrid;
  }

  @Override
  public boolean ifControlButtonsPressed(){
    for (String button : OverrideButtons) {
      if (getPressed(button)) {
        return true;
      }
    }
    return false;
  }
@Override
  public String getControlButtonPressed(){
    for (String button : OverrideButtons) {
      if (getPressed(button)) {
        return button;
      }
    }
    return null;
  }
@Override
  public boolean ifTargetingButtonsValid(){
    // if more than one targeting button is pressed, return false
    int buttonsPressed = 0;
    for (String button : TargetingButtons) {
      if (getPressed(button)) {
        buttonsPressed++;
      }
    }
    return buttonsPressed == 1;
  }
@Override
  public String getTargetingButtonPressed(){
    for (String button : TargetingButtons) {
      if (getPressed(button)) {
        return button;
      }
    }
    return null;
  }

@Override
  public void init() {
    // set all buttons to false
    for (String button : AllButtons) {
      setButton(button, false);
    }
  }
  }