package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class WristIOSolenoid implements WristIO {
  Solenoid solenoid = null;

  public void Wrist() {

    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
  }

  // TODO: figure out a way to use the constants WRIST_SOLENOID_DEPLOY and WRIST_SOLENOID_RETRACT
  @Override
  public void solenoidUp() {
    solenoid.set(true);
  }

  @Override
  public void solenoidDown() {
    solenoid.set(true);
  }
}
