// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.colors;

public class LedSetColorNoEnd extends CommandBase {
  LED leds;

  colors color;

  public LedSetColorNoEnd(LED leds, colors color) {
    this.leds = leds;
    this.color = color;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setColor(color);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
