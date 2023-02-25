package frc.robot.subsystems.led;

import frc.robot.subsystems.led.LED.colors;

// LED subsystem hardware interface.
public interface LEDIO {

  //
  public default void setColor(colors color) {}
}
