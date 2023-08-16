package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LimelightIO {
  public static class LimelightIOInputs implements LoggableInputs {

    public boolean validTarget = false;
    public double xOffset = 0;
    public double yOffset = 0;
    public double targetAreaPercent = 0;
    public double pipelineLatencyMs = 0;
    public double captureLatencyMs = 0;
    public double totalLatencyMs = 0;
    public double classID = 0;

    public void toLog(LogTable table) {
      table.put("validTarget", validTarget);
      table.put("xOffset", xOffset);
      table.put("yOffset", yOffset);
      table.put("targetAreaPercent", targetAreaPercent);
      table.put("pipelineLatencyMs", pipelineLatencyMs);
      table.put("captureLatencyMs", captureLatencyMs);
      table.put("neuralClassID", classID);
    }

    public void fromLog(LogTable table) {
      validTarget = table.getBoolean("validTarget", validTarget);
      xOffset = table.getDouble("xOffset", xOffset);
      yOffset = table.getDouble("yOffset", xOffset);
      targetAreaPercent = table.getDouble("targetAreaPercent", targetAreaPercent);
      pipelineLatencyMs = table.getDouble("pipelineLatencyMs", pipelineLatencyMs);
      captureLatencyMs = table.getDouble("captureLatencyMs", captureLatencyMs);
      classID = table.getDouble("neuralClassID", classID);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}
}
