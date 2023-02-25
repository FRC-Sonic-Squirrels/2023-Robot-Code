package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.PWMPorts;
import frc.robot.subsystems.led.LED.colors;

public class LEDIOReal implements LEDIO {

  /* Blinkin documentation https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */

  /*
   * Rev Robotics blinkinSubsystem takes a PWM signal from 1000-2000us This is identical to
   * a SparkMax motor. -1 corresponds to 1000us 0 corresponds to 1500us +1
   * corresponds to 2000us
   */
  private Spark ledController = null;
  /**
   * Creates a new blinkinSubsystem LED controller.
   *
   * @param pwmPort The PWM port the blinkinSubsystem is connected to.
   */
  public LEDIOReal() {
    ledController = new Spark(PWMPorts.kBlinkin);
  }

  @Override
  public void setColor(colors LEDcolor) {
    ledController.set(LEDcolor.colorValue);
  }
}
