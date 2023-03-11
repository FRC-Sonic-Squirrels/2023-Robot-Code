package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface WristIO {
  public static class WristIOInputs implements LoggableInputs {

    public boolean deployed = false;

    public void toLog(LogTable table) {
      table.put("Deployed", deployed);
    }

    public void fromLog(LogTable table) {
      deployed = table.getBoolean("Deployed", deployed);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Set solenoid state. */
  public default void setDeployed(boolean deployed) {}

  public default void wristUp() {}

  public default void wristDown() {}
}
