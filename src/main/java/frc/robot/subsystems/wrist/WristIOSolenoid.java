package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class WristIOSolenoid implements WristIO {
  private Solenoid solenoid;

  public void Wrist() {

    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
  }

  // TODO: figure out a way to use the constants WRIST_SOLENOID_DEPLOY and WRIST_SOLENOID_RETRACT

  public void wristUp() {
    solenoid.set(true);
  }

  public void wristDown() {
    solenoid.set(true);
  }
}
