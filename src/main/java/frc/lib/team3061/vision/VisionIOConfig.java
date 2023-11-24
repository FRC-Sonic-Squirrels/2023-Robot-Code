package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOConfig {

  public final VisionIO visionIO;
  public final String name;
  public final Transform3d robotToCamera;

  public VisionIOConfig(VisionIO visionIO, String name, Transform3d robotToCamera) {
    this.visionIO = visionIO;
    this.name = name;
    this.robotToCamera = robotToCamera;
  }
}
