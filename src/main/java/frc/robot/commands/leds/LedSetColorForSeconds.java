// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.colors;

public class LedSetColorForSeconds extends ParallelRaceGroup {

  public LedSetColorForSeconds(LED leds, colors color, double seconds) {
    super(new LedSetColor(leds, color).repeatedly(), Commands.waitSeconds(seconds));
  }
}
