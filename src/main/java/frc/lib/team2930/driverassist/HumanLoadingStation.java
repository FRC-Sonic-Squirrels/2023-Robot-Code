package frc.lib.team2930.driverassist;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;

public class HumanLoadingStation {

  PoseAndHeading[] checkpointsOutsideIn;
  PoseAndHeading finalPickupPose;

  private static HumanLoadingStation BLUE_ALLIANCE_RIGHT;
  private static HumanLoadingStation BLUE_ALLIANCE_LEFT;

  private static HumanLoadingStation RED_ALLIANCE_RIGHT;
  private static HumanLoadingStation RED_ALLIANCE_LEFT;

  private HumanLoadingStation(PoseAndHeading[] checkpoints, PoseAndHeading pickupPose) {}

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
          return RED_ALLIANCE_RIGHT.checkpointsOutsideIn;
        }

      default:
        return null;
    }
  }
}
