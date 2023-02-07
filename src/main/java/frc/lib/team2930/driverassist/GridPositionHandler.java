package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

public class GridPositionHandler {
  private static GridPositionHandler instance;

  public static GridPositionHandler getInstance() {
    if (instance == null) {
      instance = new GridPositionHandler();
    }
    return instance;
  }

  private GridPositionHandler() {}

  public static final String ROOT_TABLE = "DriverAssist/GridPosition";
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

  public static final DeadzoneBox[] allowAbleActivationAreaBlue = {
    DeadzoneBox.BLUE_COMMUNITY,
    DeadzoneBox.BLUE_ENTRANCE_WALL_SIDE,
    DeadzoneBox.BLUE_ENTRANCE_HUMAN_PLAYER_SIDE,
    DeadzoneBox.BLUE_IN_FRONT_PAD
  };

  public static final DeadzoneBox[] allowAbleActivationAreaRed = {
    DeadzoneBox.RED_COMMUNITY,
    DeadzoneBox.RED_ENTRANCE_WALL_SIDE,
    DeadzoneBox.RED_ENTRANCE_HUMAN_PLAYER_SIDE,
    DeadzoneBox.RED_IN_FRONT_PAD
  };

  private int bayIndex = 0;
  private LogicalGridLocation desiredBay = LogicalGridLocation.LOGICAL_BAY_1;

  public LogicalGridLocation getDesiredBay() {
    return desiredBay;
  }

  public void incrementNextBay() {
    if (bayIndex == logicalGridOrder.length - 1) {
      desiredBay = logicalGridOrder[logicalGridOrder.length - 1];
      return;
    }

    bayIndex++;
    desiredBay = logicalGridOrder[bayIndex];
  }

  public void decrementNextBay() {
    if (bayIndex == 0) {
      desiredBay = logicalGridOrder[0];
      return;
    }

    bayIndex--;
    desiredBay = logicalGridOrder[bayIndex];
  }

  public static PhysicalGridLocation getPhysicalLocationForLogicalBay(
      LogicalGridLocation logicalBay, Alliance alliance) {

    if (alliance == Alliance.Invalid) {
      return PhysicalGridLocation.ERROR_0_0;
    }
    if (alliance == Alliance.Red) {
      return logicalBay.redPhysical;
    } else {
      return logicalBay.bluePhysical;
    }
  }

  public static EntranceCheckpoints getEntrance(Pose2d currentPose, Alliance alliance) {
    double x = currentPose.getX();
    double y = currentPose.getY();
    if (alliance == Alliance.Blue) {
      if (y > 2.7) {
        return EntranceCheckpoints.BLUE_HUMAN_PLAYER;
      } else { // later check if in the correct band width
        return EntranceCheckpoints.BLUE_WALL;
      }
    }
    if (alliance == Alliance.Red) {
      if (y > 5.32) {
        return EntranceCheckpoints.RED_WALL;
      } else {
        return EntranceCheckpoints.RED_HUMAN_PLAYER;
      }
    }

    return EntranceCheckpoints.ERROR;
  }

  public static boolean isValidPointToStart(Pose2d currentPos, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      for (DeadzoneBox box : allowAbleActivationAreaBlue) {
        if (box.insideBox(currentPos.getTranslation())) {
          return true;
        }
      }
    }

    if (alliance == Alliance.Red) {
      for (DeadzoneBox box : allowAbleActivationAreaRed) {
        if (box.insideBox(currentPos.getTranslation())) {
          return true;
        }
      }
    }

    return false;
  }

  public static DeadzoneBox getSkipCheckpointBoxForAlliance(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return DeadzoneBox.RED_SKIP_CHECKPOINT;
    } else if (alliance == Alliance.Blue) {
      return DeadzoneBox.BLUE_SKIP_CHECKPOINT;
    } else {
      return null;
    }
  }

  public void log() {
    Logger logger = Logger.getInstance();
    logger.recordOutput("DriverAssist/GridPosition/bay_index", bayIndex);
    logger.recordOutput("DriverAssist/GridPosition/desired_bay", desiredBay.name());
  }

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
}
