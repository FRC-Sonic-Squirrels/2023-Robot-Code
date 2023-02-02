package frc.lib.team2930.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeckController {

  public void initAllButtons() {
    for (int i = 0; i < 15; i++) {
      String nameString = "/streamdeck/" + i;
      String nameStringTarget = "/streamdeck/IsTargeting" + i;
      SmartDashboard.putBoolean(nameString, false);
      SmartDashboard.putBoolean(nameStringTarget, false);
    }
  }

  public StreamDeckController() {
    initAllButtons();
  }

  private boolean GetButton(int id) {
    String nameString = "/streamdeck/" + id;
    return SmartDashboard.getBoolean(nameString, false);
  }

  public boolean GetIsTargeting(int id) {
    String nameString = "/streamdeck/IsTargeting" + id;
    return SmartDashboard.getBoolean(nameString, false);
  }

  public Trigger GetButton0() {
    return new Trigger(() -> GetButton(0));
  }

  public Trigger GetButton1() {
    return new Trigger(() -> GetButton(1));
  }

  public Trigger GetButton2() {
    return new Trigger(() -> GetButton(2));
  }

  public Trigger GetButton3() {
    return new Trigger(() -> GetButton(3));
  }

  public Trigger GetButton4() {
    return new Trigger(() -> GetButton(4));
  }

  public Trigger GetButton5() {
    return new Trigger(() -> GetButton(5));
  }

  public Trigger GetButton6() {
    return new Trigger(() -> GetButton(6));
  }

  public Trigger GetButton7() {
    return new Trigger(() -> GetButton(7));
  }

  public Trigger GetButton8() {
    return new Trigger(() -> GetButton(8));
  }

  public Trigger GetButton9() {
    return new Trigger(() -> GetButton(9));
  }

  public Trigger GetButton10() {
    return new Trigger(() -> GetButton(10));
  }

  public Trigger GetButton11() {
    return new Trigger(() -> GetButton(11));
  }

  public Trigger GetButton12() {
    return new Trigger(() -> GetButton(12));
  }

  public Trigger GetButton13() {
    return new Trigger(() -> GetButton(13));
  }

  public Trigger GetButton14() {
    return new Trigger(() -> GetButton(14));
  }
}

