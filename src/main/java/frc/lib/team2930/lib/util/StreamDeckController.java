package frc.lib.team2930.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeckController {

    static final int BUTTONS_PER_STREAM_DECK = 15;

    public void initAllButtons() {
        for (int i = 0; BUTTONS_PER_STREAM_DECK > i; i++) {
            String nameString = "/streamdeck/" + i;
            String nameStringTarget = "/streamdeck/IsTargeting" + i;
            SmartDashboard.putBoolean(nameString, false);
            SmartDashboard.putBoolean(nameStringTarget, false);
        }
    }

    public StreamDeckController() {
        initAllButtons();
    }

    private boolean getButton(int id) {
        String nameString = "/streamdeck/" + id;
        return SmartDashboard.getBoolean(nameString, false);
    }

    public boolean getIsTargeting(int id) {
        String nameString = "/streamdeck/IsTargeting" + id;
        return SmartDashboard.getBoolean(nameString, false);
    }

    public Trigger getButton0() {
        return new Trigger(() -> getButton(0));
    }

}


