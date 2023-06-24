package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionNew extends SubsystemBase {
  private final Drivetrain drivetrain;
  ArrayList<CameraResultProcessingPackage> allCameraResultProcessingPackages =
      new ArrayList<CameraResultProcessingPackage>();

  AprilTagFieldLayout aprilTagLayout;

  boolean useVisionForPoseEstimation = true;
  boolean useMaxDistanceAwayFromExistingEstimate = true;
  boolean useMaxPitchRoll = true;

  private static double MAX_ALLOWABLE_PITCH = 3;
  private static double MAX_ALLOWABLE_ROLL = 3;

  private Alert noAprilTagLayoutAlert = new Alert("No AprilTag layout file found", AlertType.ERROR);

  public VisionNew(Drivetrain drivetrain, VisionIOConfig... VisionIOConfigs) {
    this.drivetrain = drivetrain;

    try {
      aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      noAprilTagLayoutAlert.set(false);
    } catch (Exception e) {
      aprilTagLayout =
          new AprilTagFieldLayout(
              Collections.emptyList(),
              FieldConstants.FIELD_LENGTH_METERS,
              FieldConstants.FIELD_WIDTH_METERS);
      noAprilTagLayoutAlert.set(true);
    }

    for (VisionIOConfig config : VisionIOConfigs) {
      allCameraResultProcessingPackages.add(
          new CameraResultProcessingPackage(config, aprilTagLayout));
    }

    // log all robotToCamera constants, useful for cameraOverride view mode in advantage scope
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      Logger.getInstance()
          .recordOutput(
              "Vision/" + cameraPackage.name + "CameraConstant",
              new Pose3d().transformBy(cameraPackage.RobotToCamera));
    }

    for (AprilTag tag : aprilTagLayout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AllAprilTags3D/" + tag.ID, tag.pose);
    }
  }

  @Override
  public void periodic() {
    // update all inputs
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.visionIO.updateInputs(cameraPackage.visionIOInputs);
      Logger.getInstance()
          .processInputs("Vision/" + cameraPackage.name, cameraPackage.visionIOInputs);
    }

    boolean processVision = true;
    if (!useVisionForPoseEstimation) {
      processVision = false;
      setAllCameraPackageStatus(VisionProcessingStatus.NOT_PROCESSING_VISION);
    }

    if (Math.abs(drivetrain.getGyroPitch()) >= MAX_ALLOWABLE_PITCH
        || Math.abs(drivetrain.getGyroRoll()) >= MAX_ALLOWABLE_ROLL) {

      processVision = false;
      setAllCameraPackageStatus(VisionProcessingStatus.GYRO_ANGLE_NOT_VALID);
    }

    setAllCameraPackageFieldsToLog(VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES);

    if (processVision) {
      for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
        var fieldsToLog = processVision(cameraPackage);
        cameraPackage.fieldsToLog = fieldsToLog;
      }
    }

    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      logVisionProcessingFieldsAndStatus(cameraPackage);
    }

    // isconnected
    // clean logging
  }

  public VisionProcessingLoggedFields processVision(CameraResultProcessingPackage cameraPackage) {
    PhotonPipelineResult cameraResult;
    double currentResultTimeStamp;

    SwerveDrivePoseEstimator globalPoseEstimator = RobotOdometry.getInstance().getPoseEstimator();
    Pose2d prevEstimatedRobotPose = globalPoseEstimator.getEstimatedPosition();

    Pose3d newCalculatedRobotPose = null;

    double xyStandardDeviation = 0.9;
    double thetaStandardDeviation = 0.9;

    // storing fields to log
    double tagAmbiguity;
    double distanceFromTag;

    Pose3d cameraPose;

    synchronized (cameraPackage.visionIOInputs) {
      cameraResult = cameraPackage.visionIOInputs.lastResult;
      currentResultTimeStamp = cameraPackage.visionIOInputs.lastTimestamp;
    }

    if (cameraPackage.lastProcessedResultTimeStamp >= currentResultTimeStamp) {
      cameraPackage.status = VisionProcessingStatus.NOT_A_NEW_RESULT;
      return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
    }

    cameraPackage.lastProcessedResultTimeStamp = currentResultTimeStamp;

    // String ROOT_LOG_PATH = "Vision/" + cameraPackage.name + "/";

    var numTargetsSeen = cameraResult.getTargets().size();

    if (numTargetsSeen == 0) {
      cameraPackage.status = VisionProcessingStatus.NO_TARGETS_VISIBLE;
      return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
    }

    if (numTargetsSeen > 1) { // 2 or more targets
      // more than one target seen, use PNP with PV estimator
      Optional<EstimatedRobotPose> photonPoseEstimatorOptionalResult;

      cameraPackage.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      photonPoseEstimatorOptionalResult = cameraPackage.photonPoseEstimator.update(cameraResult);

      if (photonPoseEstimatorOptionalResult.isEmpty()) {
        cameraPackage.status = VisionProcessingStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY;
        return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
      }

      newCalculatedRobotPose = photonPoseEstimatorOptionalResult.get().estimatedPose;

      // TODO: work on formula for this
      xyStandardDeviation = -1;
      thetaStandardDeviation = -1;

      // logged fields
      tagAmbiguity = 0.0;
      cameraPose = newCalculatedRobotPose.transformBy(cameraPackage.RobotToCamera);

      var allSeenTags = cameraResult.getTargets();
      double totalDistance = 0.0;
      for (PhotonTrackedTarget tag : allSeenTags) {
        totalDistance +=
            aprilTagLayout
                .getTagPose(tag.getFiducialId())
                .get()
                .getTranslation()
                .getDistance(newCalculatedRobotPose.getTranslation());
      }

      distanceFromTag = totalDistance / (double) allSeenTags.size();

    } else { // 1 target

      if (cameraResult.getTargets().size() > 1) {
        cameraPackage.status = VisionProcessingStatus.LOGIC_ERROR_EXPECTED_1_TARGET;
        return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
      }

      PhotonTrackedTarget singularTag = cameraResult.getTargets().get(0);

      if (!isValidTarget(singularTag)) {
        cameraPackage.status =
            (singularTag.getPoseAmbiguity() >= VisionConstants.MAXIMUM_AMBIGUITY)
                ? VisionProcessingStatus.INVALID_TAG_AMBIGUITY_TOO_HIGH
                : VisionProcessingStatus.INVALID_TAG;
        return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
      }

      Optional<Pose3d> tagPoseOptional = aprilTagLayout.getTagPose(singularTag.getFiducialId());

      if (tagPoseOptional.isEmpty()) {
        cameraPackage.status = VisionProcessingStatus.TAG_NOT_IN_LAYOUT;
        return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
      }

      Pose3d tagPose = tagPoseOptional.get();

      Transform3d cameraToTarget = singularTag.getBestCameraToTarget();
      cameraPose = tagPose.transformBy(cameraToTarget.inverse());

      newCalculatedRobotPose = cameraPose.transformBy(cameraPackage.RobotToCamera.inverse());

      // TODO: implement standardDeviation
      xyStandardDeviation = cameraToTarget.getTranslation().getNorm() / 2.0;
      thetaStandardDeviation = -1;

      // logged fields
      tagAmbiguity = singularTag.getPoseAmbiguity();
      distanceFromTag = cameraToTarget.getTranslation().getNorm();
    }

    var distanceFromExistingPoseEstimate =
        prevEstimatedRobotPose
            .getTranslation()
            .getDistance(
                new Translation2d(newCalculatedRobotPose.getX(), newCalculatedRobotPose.getY()));

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate
            > (VisionConstants.MAX_VALID_DISTANCE_AWAY_METERS * numTargetsSeen))) {
      cameraPackage.status = VisionProcessingStatus.TOO_FAR_FROM_EXISTING_ESTIMATE;
      return VisionProcessingLoggedFields.UNSUCCESSFUL_DEFAULT_LOG_VALUES;
    }

    synchronized (globalPoseEstimator) {
      globalPoseEstimator.addVisionMeasurement(
          newCalculatedRobotPose.toPose2d(),
          currentResultTimeStamp,
          VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation));
    }

    cameraPackage.status = VisionProcessingStatus.SUCCESSFUL;
    return new VisionProcessingLoggedFields(
        numTargetsSeen,
        tagAmbiguity,
        distanceFromTag,
        distanceFromExistingPoseEstimate,
        xyStandardDeviation,
        thetaStandardDeviation,
        currentResultTimeStamp,
        cameraPose,
        newCalculatedRobotPose,
        true,
        false);

    // log num targets seen

  }

  public void setAllCameraPackageStatus(VisionProcessingStatus status) {
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.status = status;
    }
  }

  public void setAllCameraPackageFieldsToLog(VisionProcessingLoggedFields fieldsToLog) {
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.fieldsToLog = fieldsToLog;
    }
  }

  public void logVisionProcessingFieldsAndStatus(CameraResultProcessingPackage cameraPackage) {

    String ROOT_TABLE_PATH = "Vision/" + cameraPackage.name + "/";

    var fieldsToLog = cameraPackage.fieldsToLog;

    Logger logger = Logger.getInstance();

    logger.recordOutput(ROOT_TABLE_PATH + "*STATUS", cameraPackage.status.logOutput);

    logger.recordOutput(ROOT_TABLE_PATH + "calculated_robotPose_3d", fieldsToLog.robotPose3d());
    logger.recordOutput(
        ROOT_TABLE_PATH + "calculated_robotPose_2d", fieldsToLog.robotPose3d().toPose2d());
    logger.recordOutput(ROOT_TABLE_PATH + "camera_pose_3d", fieldsToLog.cameraPose());
    logger.recordOutput(ROOT_TABLE_PATH + "num_seen_targets", fieldsToLog.numSeenTargets());
    logger.recordOutput(ROOT_TABLE_PATH + "processed_timestamp", fieldsToLog.processedTimeStamp());
    logger.recordOutput(
        ROOT_TABLE_PATH + "distance_from_existing_pose_estimate",
        fieldsToLog.distanceFromExistingPoseEstimate());
    logger.recordOutput(ROOT_TABLE_PATH + "distance_from_tag", fieldsToLog.distanceFromTag());
    logger.recordOutput(ROOT_TABLE_PATH + "tag_ambiguity", fieldsToLog.tagAmbiguity());
    logger.recordOutput(
        ROOT_TABLE_PATH + "xy_standard_deviation", fieldsToLog.xyStandardDeviation());
    logger.recordOutput(
        ROOT_TABLE_PATH + "theta_standard_deviation", fieldsToLog.thetaStandardDeviation());
    logger.recordOutput(
        ROOT_TABLE_PATH + "added_vision_measurement_to_pose_estimator(SUCCESSFUL)",
        fieldsToLog.addedVisionMeasurementToPoseEstimator());
    logger.recordOutput(ROOT_TABLE_PATH + "PNP_failed", fieldsToLog.PNPFailed());
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && aprilTagLayout.getTagPose(target.getFiducialId()).isPresent();
  }

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

  private class CameraResultProcessingPackage {
    final VisionIO visionIO;
    final VisionIOInputs visionIOInputs;
    final PhotonPoseEstimator photonPoseEstimator;
    final Transform3d RobotToCamera;
    final String name;

    public VisionProcessingStatus status;
    public VisionProcessingLoggedFields fieldsToLog;
    public double lastProcessedResultTimeStamp;

    public CameraResultProcessingPackage(
        VisionIOConfig config, AprilTagFieldLayout aprilTagFieldLayout) {
      this.visionIO = config.visionIO;
      this.visionIOInputs = new VisionIOInputs();

      this.RobotToCamera = config.robotToCamera;

      this.name = config.name;

      this.photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, visionIO.getCamera(), RobotToCamera);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

      this.status = VisionProcessingStatus.UNKNOWN;
      lastProcessedResultTimeStamp = 0.0;
    }
  }

  private record VisionProcessingLoggedFields(
      double numSeenTargets,
      double tagAmbiguity,
      double distanceFromTag,
      double distanceFromExistingPoseEstimate,
      double xyStandardDeviation,
      double thetaStandardDeviation,
      double processedTimeStamp,
      Pose3d cameraPose,
      Pose3d robotPose3d,
      boolean addedVisionMeasurementToPoseEstimator,
      boolean PNPFailed) {
    public static VisionProcessingLoggedFields UNSUCCESSFUL_DEFAULT_LOG_VALUES =
        new VisionProcessingLoggedFields(
            // probably make the poses like -10, -10, 0 to move them off screen and make it obvious
            // its bad results
            -1, -1, -1, -1, -1, -1, -1, new Pose3d(), new Pose3d(), false, false);
  }

  private enum VisionProcessingStatus {
    NOT_PROCESSING_VISION(""),
    GYRO_ANGLE_NOT_VALID(""),
    NOT_A_NEW_RESULT(""),
    PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY(""),
    NO_TARGETS_VISIBLE(""),

    LOGIC_ERROR_EXPECTED_1_TARGET(""),

    TOO_FAR_FROM_EXISTING_ESTIMATE(""),

    INVALID_TAG_AMBIGUITY_TOO_HIGH(""),
    INVALID_TAG(""),

    TAG_NOT_IN_LAYOUT(""),

    SUCCESSFUL(""),
    UNKNOWN("UNKNOWN");

    public final String logOutput;

    private VisionProcessingStatus(String logOutput) {
      this.logOutput = logOutput;
    }
  }
}
