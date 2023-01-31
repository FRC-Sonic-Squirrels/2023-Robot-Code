package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface WristIO {
  public static class WristIOInputs implements LoggableInputs {

    public boolean extended = false;

    public void toLog(LogTable table) {
      table.put("Extended", extended);
    }

    public void fromLog(LogTable table) {
      extended = table.getBoolean("Extended", extended);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}

  public default void wristUp() {}
}
