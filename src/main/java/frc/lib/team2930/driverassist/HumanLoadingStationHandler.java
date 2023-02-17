package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class HumanLoadingStationHandler {

  // this checkpoint is where the elevator will extend up
  private static final double LAST_CHECKPOINT_X = 15.15;
  private static final double LEFT_SIDE_Y = 7.5;
  private static final double RIGHT_SIDE_Y = 6.1;

  private static final String ROOT_TABLE = "DriverAssist/HumanPlayerAuto";

  public final PoseAndHeading[] checkpointsOutsideIn;
  public final PoseAndHeading finalPickupPose;

  public static final BoundingBoxes[] allowableActivationAreaBlue = {
    BoundingBoxes.BLUE_HP_STATION_ACTIVATION_AREA
  };

  public static final BoundingBoxes[] allowableActivationAreaRed = {
    BoundingBoxes.RED_HP_STATION_ACTIVATION_AREA
  };

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

  public static boolean isValidPointToStart(Pose2d currentPos, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      for (BoundingBoxes box : allowableActivationAreaBlue) {
        if (box.insideBox(currentPos.getTranslation())) {
          return true;
        }
      }
    }

    if (alliance == Alliance.Red) {
      for (BoundingBoxes box : allowableActivationAreaRed) {
        if (box.insideBox(currentPos.getTranslation())) {
          return true;
        }
      }
    }

    return false;
  }

  public void log(String path, String name) {
    var logger = Logger.getInstance();

    for (int i = 0; i < checkpointsOutsideIn.length; i++) {
      logger.recordOutput(
          path + "/" + name + "/checkpointsOutsideIn/" + i, this.checkpointsOutsideIn[i].pose);
    }

    logger.recordOutput(path + "/" + name + "/finalPose/", this.finalPickupPose.pose);
  }

  public static void logAllHumanLoadingStationDriverAssist() {
    for (BoundingBoxes boundingBoxes : allowableActivationAreaBlue) {
      boundingBoxes.log(ROOT_TABLE);
    }

    for (BoundingBoxes boundingBoxes : allowableActivationAreaRed) {
      boundingBoxes.log(ROOT_TABLE);
    }

    BLUE_ALLIANCE_LEFT.log(ROOT_TABLE, "BLUE_LEFT");
    BLUE_ALLIANCE_RIGHT.log(ROOT_TABLE, "BLUE_RIGHT");

    RED_ALLIANCE_LEFT.log(ROOT_TABLE, "RED_LEFT");
    RED_ALLIANCE_RIGHT.log(ROOT_TABLE, "RED_RIGHT");
  }

  public static enum LoadingStationLocation {
    LEFT,
    RIGHT;
  }
}
