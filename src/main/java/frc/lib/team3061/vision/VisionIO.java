package frc.lib.team3061.vision;

import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult(0, new ArrayList<>());
    double lastTimestamp = 0.0;
    boolean hasNewResult = false;
    boolean connected = false;

    public void toLog(LogTable table) {
      byte[] photonPacketBytes = new byte[lastResult.getPacketSize()];
      lastResult.populatePacket(new Packet(photonPacketBytes));
      table.put("photonPacketBytes", photonPacketBytes);

      table.put("lastTimestamp", lastTimestamp);
      table.put("hasNewResult", hasNewResult);
      table.put("connected", connected);
    }

    public void fromLog(LogTable table) {
      byte[] photonPacketBytes = table.getRaw("photonPacketBytes", new byte[0]);
      lastResult = new PhotonPipelineResult();
      lastResult.createFromPacket(new Packet(photonPacketBytes));

      lastTimestamp = table.getDouble("lastTimestamp", 0.0);
      hasNewResult = table.getBoolean("hasNewResult", false);
      connected = table.getBoolean("connected", false);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  public default PhotonCamera getCamera() {
    System.out.println("------default getCamera() == null  ------------");
    return null;
  }
}
