package frc.lib.team2930.lib.util;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeckHandler {
  StreamDeckController streamDeckController = new StreamDeckController();
  public int TargetNode = 0;

  public Trigger GetIsTargeting() { // Returns if Button 0 is pressed (Targeting Button)
    return streamDeckController.GetButton0();
  }

  public void SetTarget(int num) {
    System.out.println("Updating");
    TargetNode = num;
  }

  public int GetTarget() {
    return TargetNode;
  }

  public String getTargetString() {
    return String.valueOf(TargetNode);
  }

  public void print() {
    new PrintCommand("TARGET NODE: " + TargetNode).schedule();
  }
}
