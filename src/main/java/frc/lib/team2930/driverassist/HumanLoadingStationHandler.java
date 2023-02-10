package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import java.util.ArrayList;

public class HumanLoadingStationHandler {

  // this checkpoint is where the elevator will extend up
  private static final double LAST_CHECKPOINT_X = 15.15;
  private static final double LEFT_SIDE_Y = 7.5;
  private static final double RIGHT_SIDE_Y = 6.1;

  public final PoseAndHeading[] checkpointsOutsideIn;
  public final PoseAndHeading finalPickupPose;

  private static HumanLoadingStationHandler BLUE_ALLIANCE_LEFT =
      new HumanLoadingStationHandler(
          new PoseAndHeading(
              new Pose2d(15.7, LEFT_SIDE_Y, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)),

          // checkpoints
          new PoseAndHeading(
              new Pose2d(10.35, LEFT_SIDE_Y, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)),
          new PoseAndHeading(
              new Pose2d(12.75, LEFT_SIDE_Y, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)),
          new PoseAndHeading(
              new Pose2d(LAST_CHECKPOINT_X, LEFT_SIDE_Y, Rotation2d.fromDegrees(0)),
              Rotation2d.fromDegrees(0))
          // more points later

          );

  private static HumanLoadingStationHandler BLUE_ALLIANCE_RIGHT =
      new HumanLoadingStationHandler(
          new PoseAndHeading(
              new Pose2d(15.7, RIGHT_SIDE_Y, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)),

          // checkpoints
          new PoseAndHeading(
              new Pose2d(10.35, RIGHT_SIDE_Y, Rotation2d.fromDegrees(0)),
              Rotation2d.fromDegrees(0)),
          new PoseAndHeading(
              new Pose2d(12.75, RIGHT_SIDE_Y, Rotation2d.fromDegrees(0)),
              Rotation2d.fromDegrees(0)),
          new PoseAndHeading(
              new Pose2d(LAST_CHECKPOINT_X, RIGHT_SIDE_Y, Rotation2d.fromDegrees(0)),
              Rotation2d.fromDegrees(0))
          // more points later

          );
  ;

  private static HumanLoadingStationHandler RED_ALLIANCE_RIGHT =
      new HumanLoadingStationHandler(BLUE_ALLIANCE_LEFT);
  private static HumanLoadingStationHandler RED_ALLIANCE_LEFT =
      new HumanLoadingStationHandler(BLUE_ALLIANCE_RIGHT);

  private HumanLoadingStationHandler(
      PoseAndHeading finalPickupPose, PoseAndHeading... checkpointsOutsideIn) {

    this.checkpointsOutsideIn = checkpointsOutsideIn;
    this.finalPickupPose = finalPickupPose;
  }

  private HumanLoadingStationHandler(HumanLoadingStationHandler blueToFlip) {
    this.finalPickupPose = flipForRed(blueToFlip.finalPickupPose);

    ArrayList<PoseAndHeading> checkpointsList = new ArrayList<PoseAndHeading>();

    for (PoseAndHeading poseAndHeading : blueToFlip.checkpointsOutsideIn) {
      var flipped = flipForRed(poseAndHeading);
      checkpointsList.add(flipped);
    }

    this.checkpointsOutsideIn = checkpointsList.toArray(new PoseAndHeading[0]);
  }

  private static PoseAndHeading flipForRed(PoseAndHeading toFlip) {
    var oldPose = toFlip.pose;

    Pose2d newPose =
        new Pose2d(
            oldPose.getX(),
            GridPositionHandler.FIELD_WIDTH_METERS - oldPose.getY(),
            oldPose.getRotation());

    return new PoseAndHeading(newPose, toFlip.heading);
  }

  public static enum LoadingStationLocation {
    LEFT,
    RIGHT;
  }

  public static PoseAndHeading[] getCheckpointSequenceForAlliance(
      LoadingStationLocation location, Alliance alliance) {

    switch (alliance) {
      case Red:
        if (location == LoadingStationLocation.LEFT) {
          return RED_ALLIANCE_LEFT.checkpointsOutsideIn;
        } else {
          return RED_ALLIANCE_RIGHT.checkpointsOutsideIn;
        }

      case Blue:
        if (location == LoadingStationLocation.LEFT) {
          return BLUE_ALLIANCE_LEFT.checkpointsOutsideIn;
        } else {
          return BLUE_ALLIANCE_RIGHT.checkpointsOutsideIn;
        }

      default:
        return null;
    }
  }

  public static PoseAndHeading[] checkpointsToFollow(
      Translation2d currentPose, PoseAndHeading[] sequence) {

    ArrayList<PoseAndHeading> checkpoints = new ArrayList<PoseAndHeading>();
    for (PoseAndHeading poseAndHeading : sequence) {
      if (currentPose.getX() < poseAndHeading.pose.getX()) {
        checkpoints.add(poseAndHeading);
      }
    }

    return checkpoints.toArray(new PoseAndHeading[0]);
  }

  public static PoseAndHeading getFinalPoseForLocationAndAlliance(
      LoadingStationLocation location, Alliance alliance) {
    switch (alliance) {
      case Red:
        if (location == LoadingStationLocation.LEFT) {
          return RED_ALLIANCE_LEFT.finalPickupPose;
        } else {
          return RED_ALLIANCE_RIGHT.finalPickupPose;
        }

      case Blue:
        if (location == LoadingStationLocation.LEFT) {
          return BLUE_ALLIANCE_LEFT.finalPickupPose;
        } else {
          return BLUE_ALLIANCE_RIGHT.finalPickupPose;
        }

      default:
        return null;
    }
  }

  public static void log(String path, HumanLoadingStationHandler station) {
    // Logger.getInstance()
    //     .recordOutput(path + "/BLUE_LEFT/checkpoints", station.checkpointsOutsideIn);
  }
}
