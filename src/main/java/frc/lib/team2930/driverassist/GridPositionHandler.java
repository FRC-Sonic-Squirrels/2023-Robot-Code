package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GridPositionHandler {
  // private static GridPositionHandler instance;

  // public static GridPositionHandler getInstance() {
  //   if (instance == null) {
  //     instance = new GridPositionHandler();
  //   }
  //   return instance;
  // }

  // private GridPositionHandler() {}

  // public static final String ROOT_TABLE = "DriverAssist/GridPositionAuto";
  public static final double FIELD_WIDTH_METERS = 8.02;

  public static final LogicalGridLocation[] logicalGridOrder = {
    LogicalGridLocation.LOGICAL_BAY_1,
    LogicalGridLocation.LOGICAL_BAY_2,
    LogicalGridLocation.LOGICAL_BAY_3,
    LogicalGridLocation.LOGICAL_BAY_4,
    LogicalGridLocation.LOGICAL_BAY_5,
    LogicalGridLocation.LOGICAL_BAY_6,
    LogicalGridLocation.LOGICAL_BAY_7,
    LogicalGridLocation.LOGICAL_BAY_8,
    LogicalGridLocation.LOGICAL_BAY_9,
  };

  // public static final BoundingBoxes[] allowableActivationAreaBlue = {
  //   BoundingBoxes.BLUE_COMMUNITY, BoundingBoxes.BLUE_HALF_COURT
  // };

  // public static final BoundingBoxes[] allowAbleActivationAreaRed = {
  //   BoundingBoxes.RED_COMMUNITY, BoundingBoxes.RED_HALF_COURT
  // };

  // public static PhysicalGridLocation getPhysicalLocationForLogicalBay(
  //     LogicalGridLocation logicalBay, Alliance alliance) {

  //   if (alliance == Alliance.Invalid) {
  //     return PhysicalGridLocation.ERROR_0_0;
  //   }
  //   if (alliance == Alliance.Red) {
  //     return logicalBay.redPhysical;
  //   } else {
  //     return logicalBay.bluePhysical;
  //   }
  // }

  // public static DesiredGridEntrance getClosestEntranceSide(Pose2d currentPose, Alliance alliance)
  // {
  //   var y = currentPose.getY();

  //   if (alliance == Alliance.Blue) {
  //     if (y > 2.7) {
  //       return DesiredGridEntrance.LEFT;
  //     } else {
  //       return DesiredGridEntrance.RIGHT;
  //     }
  //   }

  //   if (alliance == Alliance.Red) {
  //     if (y > 5.32) {
  //       return DesiredGridEntrance.LEFT;
  //     } else {
  //       return DesiredGridEntrance.RIGHT;
  //     }
  //   }

  //   return null;
  // }

  // public static EntranceCheckpoints getEntranceCheckpointsSpecificSide(
  //     Pose2d currentPose, DesiredGridEntrance entranceSide, Alliance alliance) {

  //   switch (entranceSide) {
  //     case LEFT:
  //       if (alliance == Alliance.Blue) {
  //         return EntranceCheckpoints.BLUE_HUMAN_PLAYER;
  //       } else {
  //         return EntranceCheckpoints.RED_WALL;
  //       }

  //     case RIGHT:
  //       if (alliance == Alliance.Blue) {
  //         return EntranceCheckpoints.BLUE_WALL;
  //       } else {
  //         return EntranceCheckpoints.RED_HUMAN_PLAYER;
  //       }

  //     default:
  //       return EntranceCheckpoints.ERROR;
  //   }
  // }

  // public static boolean isValidPointToStart(Pose2d currentPos, Alliance alliance) {
  //   if (alliance == Alliance.Blue) {
  //     for (BoundingBoxes box : allowableActivationAreaBlue) {
  //       if (box.insideBox(currentPos.getTranslation())) {
  //         return true;
  //       }
  //     }
  //   }

  //   if (alliance == Alliance.Red) {
  //     for (BoundingBoxes box : allowAbleActivationAreaRed) {
  //       if (box.insideBox(currentPos.getTranslation())) {
  //         return true;
  //       }
  //     }
  //   }

  //   return false;
  // }

  // public static boolean shouldSkipEntranceCheckpoints(
  //     Translation2d CurrentPose, Alliance alliance) {
  //   if (alliance == Alliance.Red) {
  //     return BoundingBoxes.RED_SKIP_CHECKPOINT.insideBox(CurrentPose);
  //   } else if (alliance == Alliance.Blue) {
  //     return BoundingBoxes.BLUE_SKIP_CHECKPOINT.insideBox(CurrentPose);
  //   }

  //   return false;
  // }

  public static class PoseAndHeading {
    public final Pose2d pose;
    public final Rotation2d heading;

    public PoseAndHeading(Pose2d pose, Rotation2d heading) {
      this.pose = pose;
      this.heading = heading;
    }

    public PoseAndHeading() {
      this.pose = new Pose2d();
      this.heading = new Rotation2d();
    }
  }

  // public static enum DesiredGridEntrance {
  //   LEFT,
  //   RIGHT;
  // }

  // public static void logAllGridPositionDriverAssist() {

  //   for (BoundingBoxes boundingBox : allowableActivationAreaBlue) {
  //     boundingBox.log(ROOT_TABLE);
  //   }

  //   for (BoundingBoxes boundingBox : allowAbleActivationAreaRed) {
  //     boundingBox.log(ROOT_TABLE);
  //   }

  //   EntranceCheckpoints.BLUE_HUMAN_PLAYER.log(ROOT_TABLE);
  //   EntranceCheckpoints.BLUE_WALL.log(ROOT_TABLE);

  //   EntranceCheckpoints.RED_HUMAN_PLAYER.log(ROOT_TABLE);
  //   EntranceCheckpoints.RED_WALL.log(ROOT_TABLE);

  //   for (LogicalGridLocation logicalGrid : logicalGridOrder) {
  //     logicalGrid.log(ROOT_TABLE);
  //   }
  // }
}
