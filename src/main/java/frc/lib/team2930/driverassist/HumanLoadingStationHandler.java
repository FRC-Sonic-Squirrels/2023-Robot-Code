package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import java.util.ArrayList;

public class HumanLoadingStationHandler {

  public final PoseAndHeading[] checkpointsOutsideIn;
  public final PoseAndHeading finalPickupPose;

  private static HumanLoadingStationHandler BLUE_ALLIANCE_RIGHT;
  private static HumanLoadingStationHandler BLUE_ALLIANCE_LEFT;

  private static HumanLoadingStationHandler RED_ALLIANCE_RIGHT;
  private static HumanLoadingStationHandler RED_ALLIANCE_LEFT;

  private HumanLoadingStationHandler(PoseAndHeading[] checkpoints, PoseAndHeading pickupPose) {
    checkpointsOutsideIn = null;
    finalPickupPose = null;
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

    return (PoseAndHeading[]) checkpoints.toArray();
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
}
