package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private VisionIO L_VisionIO;
  private VisionIO R_VisionIO;
  private final VisionIOInputs ioLeft = new VisionIOInputs();
  private final VisionIOInputs ioRight = new VisionIOInputs();
  private AprilTagFieldLayout layout;
  private DriverStation.Alliance lastAlliance = DriverStation.Alliance.Invalid;

  private double lastTimestampLeft;
  private double lastTimestampRight;
  private SwerveDrivePoseEstimator poseEstimator;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  public Vision(SwerveDrivePoseEstimator poseEstimator, VisionIO L_VisionIO, VisionIO R_VisionIO) {
    this.L_VisionIO = L_VisionIO;
    this.R_VisionIO = R_VisionIO;
    this.poseEstimator = poseEstimator;

    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  public enum Camera {
    LEFT,
    RIGHT;
  }

  public double getLatestTimestamp(Camera camera) {
    switch (camera) {
      case LEFT:
        return ioLeft.lastTimestamp;
      case RIGHT:
        return ioRight.lastTimestamp;
      default:
        return 0;
    }
  }

  public PhotonPipelineResult getLatestResult(Camera camera) {
    switch (camera) {
      case LEFT:
        return ioLeft.lastResult;
      case RIGHT:
        return ioRight.lastResult;
    }
    return null;
  }

  @Override
  public void periodic() {

    L_VisionIO.updateInputs(ioLeft);
    R_VisionIO.updateInputs(ioRight);
    Logger.getInstance().processInputs("Vision/Left", ioLeft);
    Logger.getInstance().processInputs("Vision/Right", ioRight);

//    if (DriverStation.getAlliance() != lastAlliance) {
//      lastAlliance = DriverStation.getAlliance();
//      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
//        layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
//      } else {
//        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
//      }
//    }

    if (lastTimestampLeft < getLatestTimestamp(Camera.LEFT)) {
      lastTimestampLeft = getLatestTimestamp(Camera.LEFT);
      updatePose(Camera.LEFT);
    }
    if (lastTimestampRight < getLatestTimestamp(Camera.RIGHT)) {
      lastTimestampRight = getLatestTimestamp(Camera.RIGHT);
      updatePose(Camera.RIGHT);
    }
  }

  private void updatePose(Camera camera) {
    // TODO: test to see if we only need the best target rather than every target:
    // getLatestResult(camera).getBestTarget()
    for (PhotonTrackedTarget target : getLatestResult(camera).getTargets()) {
      if (isValidTarget(target)) {
        // photon camera to target
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        // the raw json position of target
        // probably optional to avoid crashes from io errors
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isEmpty()) {
          break; // this would only be "return" if the for loop has getBestTarget()
        }

        // tag pose from json
        Pose3d tagPose = tagPoseOptional.get();
        // the camera position, transforms the json tag
        // by the inverse of photons cameraToTarget. CameraToTarget inverse is TargetToCamera
        Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
        // camera might not be in the center of the robot
        Pose3d robotPose;
        switch (camera) {
          case LEFT:
            robotPose = cameraPose.transformBy(VisionConstants.LEFT_ROBOT_TO_CAMERA.inverse());
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), getLatestTimestamp(camera));

            Logger.getInstance().recordOutput("Vision/Left/TagPose", tagPose);
            Logger.getInstance().recordOutput("Vision/Left/CameraPose", cameraPose);
            Logger.getInstance().recordOutput("Vision/Left/RobotPose", robotPose.toPose2d());

          case RIGHT:
            robotPose = cameraPose.transformBy(VisionConstants.RIGHT_ROBOT_TO_CAMERA.inverse());
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), getLatestTimestamp(camera));

            Logger.getInstance().recordOutput("Vision/Right/TagPose", tagPose);
            Logger.getInstance().recordOutput("Vision/Right/CameraPose", cameraPose);
            Logger.getInstance().recordOutput("Vision/Right/RobotPose", robotPose.toPose2d());
          default:
            robotPose = null;
        }
      }
    }
  }

  public boolean tagVisible(int id, Camera camera) {
    PhotonPipelineResult result = getLatestResult(camera);
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
  // Find the average of the two transforms by getting the mean of this method
  // with both cameras in the user implementation of the code
  public Transform3d getTransform3dToTag(int id, Camera camera) {
    Transform3d bestTransform3d = null;

    PhotonPipelineResult result = getLatestResult(camera);

    for (PhotonTrackedTarget target : result.getTargets()) {
      // the target must be the same as the given target id, and the target must be available
      if (target.getFiducialId() != id || !isValidTarget(target)) {
        break;
      }
      switch (camera) {
        case LEFT:
          bestTransform3d =
              VisionConstants.LEFT_ROBOT_TO_CAMERA.plus(target.getBestCameraToTarget());

        case RIGHT:
          bestTransform3d =
              VisionConstants.RIGHT_ROBOT_TO_CAMERA.plus(target.getBestCameraToTarget());
      }
    }
    return bestTransform3d;
  }

  public Rotation2d getAngleToTag(int id, Camera camera) {
    Transform3d transform = getTransform3dToTag(id, camera);
    if (transform != null) {
      return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
    } else {
      return null;
    }
  }

  public double getDistanceToTag(int id, Camera camera) {
    Transform3d transform = getTransform3dToTag(id, camera);
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
