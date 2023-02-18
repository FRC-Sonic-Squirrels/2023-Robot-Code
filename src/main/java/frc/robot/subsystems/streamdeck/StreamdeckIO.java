package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface StreamdeckIO {
  public static class StreamdeckInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub

    }

    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub

    }
  }

  default Trigger getButton(String buttonNumber) {
    return new Trigger(() -> SmartDashboard.getBoolean("/streamdeck/" + buttonNumber, false));
  }

  public default void setButtonSprite(int buttonNumber) {}
}
