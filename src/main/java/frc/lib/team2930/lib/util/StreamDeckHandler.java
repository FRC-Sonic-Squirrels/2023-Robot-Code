package frc.lib.team2930.lib.util;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeckHandler {
    StreamDeckController streamDeckController = new StreamDeckController();
    public int targetNode; // Base is 0

    public Trigger getIsTargeting() { // Returns if Button 0 is pressed (Targeting Button)
        return streamDeckController.getButton0();
    }

    public void setTarget(int num) {
        System.out.println("Updating");
        targetNode = num;
    }

    public int getTarget() {
        return targetNode;
    }

    public String getTargetString() {
        return String.valueOf(targetNode);
    }

    public void print() {
        new PrintCommand("TARGET NODE: " + targetNode).schedule();
    }
}
