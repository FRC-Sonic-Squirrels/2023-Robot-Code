// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import java.util.Objects;

public class Streamdeck extends SubsystemBase {

  StreamdeckReal Deck = new StreamdeckReal();
  boolean overrideCone = false;
  boolean overrideCube = false;

  public Streamdeck() {
    Commands.print("CREATED").schedule();
  }

  @Override
  public void periodic() {
    if (Deck.ifControlButtonsPressed()) {
      if (Objects.equals(Deck.getControlButtonPressed(), "0_0")) {
        if (Deck.ifTargetingButtonsValid()) {
          if (Deck.getDesiredFromButton(Deck.getTargetingButtonPressed()) != RobotState.getInstance().getDesiredLogicalGrid()) {
            Deck.setDesiredFromButton(Deck.getTargetingButtonPressed());
          }
        }
      }
    }
    overrideCone = Objects.equals(Deck.getControlButtonPressed(), "1_0");
    overrideCube = Objects.equals(Deck.getControlButtonPressed(), "2_0");

    SmartDashboard.putBoolean("Override Cone", overrideCone);
    SmartDashboard.putBoolean("Override Cube", overrideCube);
  }
}
