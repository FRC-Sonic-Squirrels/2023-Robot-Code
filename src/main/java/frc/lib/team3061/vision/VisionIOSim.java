package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {
  private static final double DIAGONAL_FOV = 70; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 720; // image heigh in px
  private static final double MIN_TARGET_AREA =
      500.0; // change if width/height changes //TODO: Find a good number for this
  private final PhotonCamera camera;

  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  private Supplier<Pose2d> poseSupplier;
  private SimVisionSystem simVision;
  private AprilTagFieldLayout layout;

  public VisionIOSim(
      AprilTagFieldLayout layout,
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToCamera,
      String networkName) {

    this.camera = new PhotonCamera(networkName);

    this.poseSupplier = poseSupplier;
    this.layout = layout;
    this.simVision =
        new SimVisionSystem(
            networkName,
            DIAGONAL_FOV,
            robotToCamera, // .inverse(),
            18000, // TODO: FIND A GOOD NUMBER FOR THIS
            IMG_WIDTH,
            IMG_HEIGHT,
            MIN_TARGET_AREA);

    updateSimInputs();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /*
     * based on https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
     * and https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub =
        inst.getTable("/photonvision/" + networkName)
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000.0);
          synchronized (VisionIOSim.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    this.simVision.processFrame(poseSupplier.get());

    inputs.lastTimestamp = this.lastTimestamp;
    inputs.lastResult = this.lastResult;
    inputs.connected = camera.isConnected();
  }

  private void updateSimInputs() {
    this.simVision.clearVisionTargets();
    for (AprilTag tag : layout.getTags()) {
      this.simVision.addSimVisionTarget(
          new SimVisionTarget(tag.pose, Units.inchesToMeters(6), Units.inchesToMeters(6), tag.ID));
    }
  }

  @Override
  public PhotonCamera getCamera() {
    return camera;
  }
}
