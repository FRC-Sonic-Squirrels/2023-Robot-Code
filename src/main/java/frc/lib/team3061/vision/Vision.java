package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private VisionIO visionIO;
  private final VisionIOInputs io = new VisionIOInputs();
  private AprilTagFieldLayout layout;
  private DriverStation.Alliance lastAlliance = DriverStation.Alliance.Invalid;

  private double lastTimestamp;
  private SwerveDrivePoseEstimator poseEstimator;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  public double getLatestTimestamp() {
    return io.lastTimestamp;
  }

  public PhotonPipelineResult getLatestResult() {
    return io.lastResult;
  }

  @Override
  public void periodic() {

    visionIO.updateInputs(io);
    Logger.getInstance().processInputs("Vision", io);

    // TODO: figure out how this affects our position and how this effects auto paths
    // where is the red alliance origin? top right? bottom right?
    if (DriverStation.getAlliance() != lastAlliance) {
      lastAlliance = DriverStation.getAlliance();
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      } else {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
    }

    if (lastTimestamp < getLatestTimestamp()) {
      lastTimestamp = getLatestTimestamp();
      for (PhotonTrackedTarget target : getLatestResult().getTargets()) {
        if (isValidTarget(target)) {
          // photon camera to target
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          // the raw json position of target
          // probably optional to avoid crashes from io errors
          Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
          if (tagPoseOptional.isPresent()) {
            // tag pose from json
            Pose3d tagPose = tagPoseOptional.get();
            // the camera position, transforms the json tag
            // by the inverse of photons cameraToTarget. CameraToTarget inverse is TargetToCamera
            Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
            // camera might not be in the center of the robot
            Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse());
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), getLatestTimestamp());

            Logger.getInstance().recordOutput("Vision/TagPose", tagPose);
            Logger.getInstance().recordOutput("Vision/CameraPose", cameraPose);
            Logger.getInstance().recordOutput("Vision/RobotPose", robotPose.toPose2d());
          }
        }
      }
    }
  }

  public boolean tagVisible(int id) {
    PhotonPipelineResult result = getLatestResult();
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id && isValidTarget(target)) {
        return true;
      }
    }
    return false;
  }

  /**
   * returns the best Rotation3d from the robot to the given target.
   *
   * @param id
   * @return the Transform3d or null if there isn't
   */
  public Transform3d getTransform3dToTag(int id) {
    PhotonPipelineResult result = getLatestResult();
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id && isValidTarget(target)) {
        return VisionConstants.ROBOT_TO_CAMERA.plus(target.getBestCameraToTarget());
      }
    }
    return null;
  }

  public Rotation2d getAngleToTag(int id) {
    Transform3d transform = getTransform3dToTag(id);
    if (transform != null) {
      return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
    } else {
      return null;
    }
  }

  public double getDistanceToTag(int id) {
    Transform3d transform = getTransform3dToTag(id);
    if (transform != null) {
      return transform.getTranslation().toTranslation2d().getNorm();
    } else {
      return -1;
    }
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }
}
