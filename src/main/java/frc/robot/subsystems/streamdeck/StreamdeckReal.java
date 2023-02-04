// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.streamdeck.Streamdeck.StreamDeckLocation;

/** Add your docs here. */
public class StreamdeckReal implements StreamdeckIO {

  public StreamdeckReal(StreamDeckLocation location) {
    if (location == StreamDeckLocation.LEFT) {
      // init for the NT with the left stream deck as the root directory
    } else {
      // init for the NT with the right stream deck as the root directory
    }
  }

  @Override
  public Trigger getButton(int buttonNumber) {
    // TODO
    return null;
  }

  @Override
  public void setButtonSprite(int buttonNumber) {
    // TODO
  }
}
