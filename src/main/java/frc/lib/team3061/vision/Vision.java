package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.autonomous.SwerveAutos;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private VisionIO L_VisionIO;
  private VisionIO R_VisionIO;
  private final VisionIOInputs ioLeft = new VisionIOInputs();
  private final VisionIOInputs ioRight = new VisionIOInputs();
  private AprilTagFieldLayout layout;

  private HashMap<String, Double> lastTimestamp = new HashMap<>();

  PhotonPoseEstimator leftPhotonPoseEstimator;
  PhotonPoseEstimator rightPhotonPoseEstimator;

  private boolean updatePoseWithVisionReadings = true;
  private boolean useMaxValidDistanceAway = true;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  private Drivetrain drivetrain;

  private static double MAX_ALLOWABLE_PITCH = 3;
  private static double MAX_ALLOWABLE_ROLL = 3;

  public Vision(VisionIO L_VisionIO, VisionIO R_VisionIO, Drivetrain drivetrain) {
    this.L_VisionIO = L_VisionIO;
    this.R_VisionIO = R_VisionIO;

    this.drivetrain = drivetrain;

    Logger.getInstance().recordOutput("Vision/updatePoseWithVisionReadings", true);
    Logger.getInstance().recordOutput("Vision/useMaxValidDistanceAway", true);

    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout =
          new AprilTagFieldLayout(
              new ArrayList<>(), SwerveAutos.FIELD_LENGTH_METERS, SwerveAutos.FIELD_WIDTH_METERS);
      noAprilTagLayoutAlert.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }

    Logger.getInstance()
        .recordOutput(
            "Vision/LeftCameraConstant",
            new Pose3d().transformBy(VisionConstants.LEFT_ROBOT_TO_CAMERA));
    Logger.getInstance()
        .recordOutput(
            "Vision/RightCameraConstant",
            new Pose3d().transformBy(VisionConstants.RIGHT_ROBOT_TO_CAMERA));

    PoseStrategy strategy = PoseStrategy.MULTI_TAG_PNP;
    if (Robot.isSimulation()) {
      // MULTI_TAG_PNP doesn't work in simulation
      strategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
    }

    lastTimestamp.put("Left", 0.0);
    lastTimestamp.put("Right", 0.0);

    leftPhotonPoseEstimator =
        new PhotonPoseEstimator(
            layout, strategy, L_VisionIO.getCamera(), VisionConstants.LEFT_ROBOT_TO_CAMERA);
    leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    rightPhotonPoseEstimator =
        new PhotonPoseEstimator(
            layout, strategy, R_VisionIO.getCamera(), VisionConstants.RIGHT_ROBOT_TO_CAMERA);
    rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void periodic() {

    L_VisionIO.updateInputs(ioLeft);
    R_VisionIO.updateInputs(ioRight);
    Logger.getInstance().processInputs("Vision/Left", ioLeft);
    Logger.getInstance().processInputs("Vision/Right", ioRight);

    if (!updatePoseWithVisionReadings) {
      return;
    }

    if (Math.abs(drivetrain.getGyroPitch()) >= MAX_ALLOWABLE_PITCH
        || Math.abs(drivetrain.getGyroRoll()) >= MAX_ALLOWABLE_ROLL) {

      Logger.getInstance().recordOutput("Vision/ValidGyroAngle", false);
      return;
    }

    Logger.getInstance().recordOutput("Vision/ValidGyroAngle", true);

    updatePose(
        L_VisionIO, ioLeft, leftPhotonPoseEstimator, VisionConstants.LEFT_ROBOT_TO_CAMERA, "Left");
    updatePose(
        R_VisionIO,
        ioRight,
        rightPhotonPoseEstimator,
        VisionConstants.RIGHT_ROBOT_TO_CAMERA,
        "Right");
  }

  private void updatePose(
      VisionIO visionIO,
      VisionIOInputs io,
      PhotonPoseEstimator photonPoseEstimator,
      Transform3d cameraToRobot,
      String name) {

    boolean updated = false;
    boolean pnpFailed = false;

    SwerveDrivePoseEstimator poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    Pose2d prevEstimatedRobotPose =
        RobotOdometry.getInstance().getPoseEstimator().getEstimatedPosition();
    double timestamp = 0;
    double distance = 0.0;
    PhotonPipelineResult cameraResult = null;
    Pose3d robotPose = null;

    // Synchronize read/write on camera data, this is getting written asynchronously
    synchronized (io) {
      cameraResult = io.lastResult;
      timestamp = io.lastTimestamp;
    }

    // is this a new camera result?
    double prevTimestamp = lastTimestamp.get(name);
    if (prevTimestamp >= timestamp) {
      return;
    }
    lastTimestamp.put(name, timestamp);

    int targetsSeen = cameraResult.getTargets().size();
    Logger.getInstance().recordOutput("Vision/" + name + "/SeenTargets", targetsSeen);

    if (targetsSeen > 1) {
      // more than one target seen, use PNP with PV estimator
      Optional<EstimatedRobotPose> result;
      EstimatedRobotPose estimatedRobotPose = null;

      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      result = photonPoseEstimator.update(cameraResult);
      if (result.isPresent()) {
        estimatedRobotPose = result.get();
      }

      if (estimatedRobotPose != null) {
        robotPose = estimatedRobotPose.estimatedPose;
        Logger.getInstance()
            .recordOutput("Vision/" + name + "/CameraPose", robotPose.transformBy(cameraToRobot));
      } else {
        pnpFailed = true;
      }
      Logger.getInstance().recordOutput("Vision/" + name + "/Ambiguity", 0.0);

    } else {
      // zero or 1 target, manually check if it is accurate enough
      for (PhotonTrackedTarget target : cameraResult.getTargets()) {
        if (isValidTarget(target)) {
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
          if (tagPoseOptional.isEmpty()) {
            break;
          }

          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());

          robotPose = cameraPose.transformBy(cameraToRobot.inverse());

          Logger.getInstance()
              .recordOutput("Vision/" + name + "/Ambiguity", target.getPoseAmbiguity());
          Logger.getInstance().recordOutput("Vision/" + name + "/CameraPose", cameraPose);
        }
      }
    }

    if (robotPose != null) {
      Logger.getInstance().recordOutput("Vision/" + name + "/DistanceFromRobot", distance);
      Logger.getInstance().recordOutput("Vision/" + name + "/RobotPose", robotPose.toPose2d());

      distance =
          prevEstimatedRobotPose
              .getTranslation()
              .getDistance(new Translation2d(robotPose.getX(), robotPose.getY()));

      // distance from vision estimate to last position estimate
      if ((useMaxValidDistanceAway)
          && (distance > VisionConstants.MAX_VALID_DISTANCE_AWAY_METERS)) {
        // no update
      } else {
        // we passed all the checks, update the pose
        updated = true;
        poseEstimator.addVisionMeasurement(robotPose.toPose2d(), timestamp);
      }
    }

    Logger.getInstance().recordOutput("Vision/" + name + "/Updated", updated);
    Logger.getInstance().recordOutput("Vision/" + name + "/pnpFailed", pnpFailed);
  }

  public boolean tagVisible(int id, PhotonPipelineResult result) {
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
  public Transform3d getTransform3dToTag(
      int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d bestTransform3d = null;

    for (PhotonTrackedTarget target : result.getTargets()) {
      // the target must be the same as the given target id, and the target must be available
      if (target.getFiducialId() != id || !isValidTarget(target)) {
        break;
      }

      bestTransform3d = robotToCamera.plus(target.getBestCameraToTarget());
    }
    return bestTransform3d;
  }

  public Rotation2d getAngleToTag(int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d transform = getTransform3dToTag(id, result, robotToCamera);
    if (transform != null) {
      return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
    } else {
      return null;
    }
  }

  public double getDistanceToTag(int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d transform = getTransform3dToTag(id, result, robotToCamera);
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

  public void enableMaxDistanceAwayForTags() {
    Logger.getInstance().recordOutput("Vision/useMaxValidDistanceAway", true);
    useMaxValidDistanceAway = true;
  }

  public void disableMaxDistanceAwayForTags() {
    Logger.getInstance().recordOutput("Vision/useMaxValidDistanceAway", false);
    useMaxValidDistanceAway = false;
  }

  public void enableUpdatePoseWithVisionReadings() {
    Logger.getInstance().recordOutput("Vision/updatePoseWithVisionReadings", true);
    updatePoseWithVisionReadings = true;
  }

  public void disableUpdatePoseWithVisionReadings() {
    Logger.getInstance().recordOutput("Vision/updatePoseWithVisionReadings", false);
    updatePoseWithVisionReadings = false;
  }
}

class TargetComparator implements java.util.Comparator<PhotonTrackedTarget> {
  @Override
  public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
    double ambiguityDiff = a.getPoseAmbiguity() - b.getPoseAmbiguity();
    if (ambiguityDiff < 0.0) {
      return -1;
    }
    if (ambiguityDiff > 0.0) {
      return 1;
    }
    return 0;
  }
}
