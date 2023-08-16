package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tl = table.getEntry("tl");
  NetworkTableEntry cl = table.getEntry("cl");
  NetworkTableEntry tclass = table.getEntry("tclass");

  public LimelightIOReal() {
    table.getEntry("ledMode").setNumber(1);
    table.getEntry("camMode").setNumber(0);
    table.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.validTarget = tv.getDouble(0.0) != 0;
    inputs.xOffset = tx.getDouble(0.0);
    inputs.yOffset = ty.getDouble(0.0);
    inputs.targetAreaPercent = ta.getDouble(0.0);
    inputs.pipelineLatencyMs = tl.getDouble(0.0);
    inputs.totalLatencyMs = ta.getDouble(0.0) + tl.getDouble(0.0);
    inputs.classID = tclass.getDouble(0.0);
  }

  @Override
  public void ledMode(double mode) {
    table.getEntry("ledMode").setNumber(mode);
  }
}
