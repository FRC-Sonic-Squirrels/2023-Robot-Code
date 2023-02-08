// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.streamdeck.StreamdeckIO.StreamdeckInputs;

public class Streamdeck extends SubsystemBase {
  /** Creates a new Streamdeck. */
  StreamdeckIO leftIO;

  StreamdeckIO rightIO;

  StreamdeckInputs leftInputs = new StreamdeckInputs();
  StreamdeckInputs rightInputs = new StreamdeckInputs();

  public enum TargetingMode {
    TARGET, GREYING, DE_GREY, NONE
  }

  public boolean[] targetingButtonsHeld = {false, false, false};

  TargetingMode targetingMode  = TargetingMode.NONE;

  public Streamdeck(StreamdeckIO leftIO, StreamdeckIO rightIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;
  }

  public boolean[] getAllTargetable() {
    String[] buttons = {"0_1", "0_2", "0_3", "0_4", "0_5", "0_6", "0_7", "0_8", "0_9", "1_1", "1_2", "1_3", "1_4", "1_5", "1_6", "1_7", "1_8", "1_9",
             "2_1", "2_2", "2_3", "2_4", "2_5", "2_6", "2_7", "2_8", "2_9"};

    int buttonAmount = buttons.length;
    boolean[] buttonsHeld = new boolean[buttonAmount];
    for (int i = 0; i < buttonAmount; i++) {
      buttonsHeld[i] = leftIO.getButton(buttons[i]).getAsBoolean();
    }
    return buttonsHeld;
  }

  public boolean[] getTargetingButtonsHeld() {
    String[] buttons = {"0_0", "1_0", "2_0"};
    int buttonAmount = buttons.length;
    boolean[] buttonsHeld = new boolean[buttonAmount];
    for (int i = 0; i < buttonAmount; i++) {
      buttonsHeld[i] = leftIO.getButton(buttons[i]).getAsBoolean();
    }
    return buttonsHeld;
  }

  public boolean isTargetingValid(boolean[] listOfButtons) {
    // If only one button is pressed, then return true
    int trueCount = 0;
    for (boolean listOfButton : listOfButtons) {
      if (listOfButton) {
        trueCount++;
      }
    }
    return trueCount == 1;
  }

  public String findButtonPressed() {
    String[] buttons = {"0_1", "0_2", "0_3", "0_4", "0_5", "0_6", "0_7", "0_8", "0_9", "1_1", "1_2", "1_3", "1_4", "1_5", "1_6", "1_7", "1_8", "1_9",
             "2_1", "2_2", "2_3", "2_4", "2_5", "2_6", "2_7", "2_8", "2_9"};

    int buttonAmount = buttons.length;
    boolean[] buttonsHeld = new boolean[buttonAmount];
    for (int i = 0; i < buttonAmount; i++) {
      buttonsHeld[i] = leftIO.getButton(buttons[i]).getAsBoolean();
      if (buttonsHeld[i]) {
        return buttons[i];
      }
    }
    return "NONE";
  }

  @Override
  public void periodic() {
    targetingButtonsHeld[0] = leftIO.getButton("0_0").getAsBoolean();
    targetingButtonsHeld[1] = leftIO.getButton("1_0").getAsBoolean();
    targetingButtonsHeld[2] = leftIO.getButton("2_0").getAsBoolean();

    if (isTargetingValid(targetingButtonsHeld)) {
      if (targetingButtonsHeld[0]) {
        if (targetingMode == TargetingMode.NONE) {
          targetingMode = TargetingMode.TARGET;
          Commands.print("TargetingButton").schedule();
        }
      } else if (targetingButtonsHeld[1]) {
        if (targetingMode == TargetingMode.NONE) {
          targetingMode = TargetingMode.GREYING;
          Commands.print("GreyingButton").schedule();
        }
      } else if (targetingButtonsHeld[2]) {
        if (targetingMode == TargetingMode.NONE) {
          targetingMode = TargetingMode.DE_GREY;
          Commands.print("De-greyingButton").schedule();
        }
      }
    } else {
      if (targetingMode != TargetingMode.NONE) {
        targetingMode = TargetingMode.NONE;
        Commands.print("NONE").schedule();
      }
    }

    if (targetingMode == TargetingMode.TARGET) {
      if (isTargetingValid(getAllTargetable())) {
        // Do targeting stuff
        String buttonPressed = findButtonPressed();
        Commands.print("TargetingAction" + buttonPressed).schedule();
        SmartDashboard.putString("/streamdeck/TargetingSet", buttonPressed);
      }
    } else if (targetingMode == TargetingMode.GREYING) {
      if (isTargetingValid(getAllTargetable())) {
        // Do grey stuff
        String buttonPressed = findButtonPressed();
        Commands.print("GreyingAction" + buttonPressed).schedule();
      }
    } else if (targetingMode == TargetingMode.DE_GREY) {
      if (isTargetingValid(getAllTargetable())) {
        // Do de greying stuff
        String buttonPressed = findButtonPressed();
        Commands.print("De-greyingAction" + buttonPressed).schedule();
      }
    }


    // Idea:
    // One array will store the values of the first colum of buttons, these are "action buttons" as
    // they decide the behavior of all the other buttons.
    // for example: if [0] is true for example targeting more is true, if [1] is true then fill in
    // mode is true
    // if multiple values from the array are true then the current behavior mode is NONE
    // I would create an enum called BehaviorMode, which would be either Targeting, FillIn, UnScore,
    // None
    // use a switch statement in the periodic to do different behavior depending on the current mode

    // building more on the array of values idea you can make a big 2d array which spans the rest of
    // the 3 rows and 9 column
    // have a helper method which would fill this array in with values from both stream decks.

    // similar logic to the action buttons array, if 2 or more buttons are active then don't do
    // anything
    // if one button is active then you can the current selected button row and colum variables to
    // the ones corresponding to the button
    // use those variables and the switch statement with the modes to determine what to do

    // you will have to create helper methods for turning a row and colum position into a: which
    // stream deck and which button ID of that specific stream deck and vise versa for this system

    // imo this is a lot cleaner than assigning every streamdeck button a trigger which runs a
    // corresponding
    // command, that would get really repetitive and hard to maintain and debug. This seems like a
    // cleaner solution to me

    // you can also have a 2d array for the current scored pieces, this will allow you to do logic
    // for automatically determining for example

    // if you dont want to deal with 2d arrays you can make your own custom data structure
    // maybe something like StreamdeckMatrix which could have 3 variable arrays for each row
    // the advantage of this would be isolating all the matrix logic for getting, setting, and
    // working
    // with a matrix across 2 different devices, as apposed to having a bunch of functions in this
    // subsystem

    // using a 2d array or a custom data structure (could be something different from the example I
    // gave) are both valid solutions to this problem, pick what you wanna work with

    // ofc these are just my ideas if you have thoughts on others way to tackle this issue share
    // them

    // This method will be called once per scheduler run
  }

  public enum StreamDeckLocation {
    LEFT,
    RIGHT
  }
}
