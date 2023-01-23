package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class GridPositionHandler {

  public static final String ROOT_TABLE = "DriverAssist/GridPosition";

  private static final LogicalGridLocation[] logicalGridOrder = {
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
    DeadzoneBox.BLUE_COMMUNITY_PART_2,
    DeadzoneBox.BLUE_ENTRANCE_HUMAN_PLAYER_SIDE
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

  public static EntranceCheckpoint getEntrance(Pose2d currentPose, Alliance alliance) {
    double x = currentPose.getX();
    double y = currentPose.getY();
    if (alliance == Alliance.Blue) {
      if (y > 2.7) {
        return EntranceCheckpoint.BLUE_HUMAN_PLAYER;
      } else { // later check if in the correct band width
        return EntranceCheckpoint.BLUE_WALL;
      }
    }
    if (alliance == Alliance.Red) {
      if (y > 5.2) {
        return EntranceCheckpoint.RED_WALL;
      } else {
        return EntranceCheckpoint.RED_HUMAN_PLAYER;
      }
    }

    // FIXME should not be null
    return null;
  }

  public static boolean isValidPointToStart(Pose2d currentPos, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      for (DeadzoneBox box : allowAbleActivationAreaBlue) {
        if (box.insideBox(currentPos.getTranslation())) {
          return true;
        }
      }
      return false;
    }

    // FIX ME: numbers for red alliance
    return true;
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
  }

  public enum PhysicalGridLocation {
    BLUE_PHYSICAL_BAY_1(
        new PoseAndHeading(
            new Pose2d(2.0, 4.96, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 4.96, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_2(
        new PoseAndHeading(
            new Pose2d(2.0, 4.45, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 4.45, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_3(
        new PoseAndHeading(
            new Pose2d(2.0, 3.87, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 3.87, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_4(
        new PoseAndHeading(
            new Pose2d(2.0, 3.3, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 3.3, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_5(
        new PoseAndHeading(
            new Pose2d(2.0, 2.75, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 2.75, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_6(
        new PoseAndHeading(
            new Pose2d(2.0, 2.2, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 2.2, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_7(
        new PoseAndHeading(
            new Pose2d(2.0, 1.6, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 1.6, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_8(
        new PoseAndHeading(
            new Pose2d(2.0, 1.05, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 1.05, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    BLUE_PHYSICAL_BAY_9(
        new PoseAndHeading(
            new Pose2d(2.0, 0.52, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 0.52, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    // ----------------------------RED BAYS--------------------------------------
    RED_PHYSICAL_BAY_1(
        new PoseAndHeading(
            new Pose2d(2.0, 7.5, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 7.5, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_2(
        new PoseAndHeading(
            new Pose2d(2.0, 6.95, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 6.95, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_3(
        new PoseAndHeading(
            new Pose2d(2.0, 6.4, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 6.4, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_4(
        new PoseAndHeading(
            new Pose2d(2.0, 5.84, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 5.84, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_5(
        new PoseAndHeading(
            new Pose2d(2.0, 5.27, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 5.27, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_6(
        new PoseAndHeading(
            new Pose2d(2.0, 4.73, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 4.73, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_7(
        new PoseAndHeading(
            new Pose2d(2.0, 4.17, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 4.17, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_8(
        new PoseAndHeading(
            new Pose2d(2.0, 3.6, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 3.6, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    RED_PHYSICAL_BAY_9(
        new PoseAndHeading(
            new Pose2d(2.0, 3.07, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180)),
        new PoseAndHeading(
            new Pose2d(1.8, 3.07, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(180))),

    ERROR_0_0(
        new PoseAndHeading(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)),
        new PoseAndHeading(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)));

    public final PoseAndHeading lineup;
    public final PoseAndHeading score;

    private PhysicalGridLocation(PoseAndHeading lineup, PoseAndHeading score) {
      this.lineup = lineup;
      this.score = score;
    }
  }

  public enum EntranceCheckpoint {
    // TODO: Add a to pathpoint function(add this to the other physical grid locations as well)
    BLUE_WALL(
        new PoseAndHeading(
            new Pose2d(2.5, 0.7, Rotation2d.fromDegrees(0)),
            Rotation2d.fromDegrees(180))), // x:2.17

    BLUE_HUMAN_PLAYER(
        new PoseAndHeading(
            new Pose2d(2.5, 4.64, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(180))),

    RED_WALL(
        new PoseAndHeading(
            new Pose2d(2.5, 7.18, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(180))),

    RED_HUMAN_PLAYER(
        new PoseAndHeading(
            new Pose2d(2.5, 3.4, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(180)));

    public final PoseAndHeading location;

    private EntranceCheckpoint(PoseAndHeading location) {
      this.location = location;
    }
  }

  public enum LogicalGridLocation {
    LOGICAL_BAY_1(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_1, PhysicalGridLocation.RED_PHYSICAL_BAY_1),

    LOGICAL_BAY_2(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_2, PhysicalGridLocation.RED_PHYSICAL_BAY_2),

    LOGICAL_BAY_3(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_3, PhysicalGridLocation.RED_PHYSICAL_BAY_3),

    LOGICAL_BAY_4(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_4, PhysicalGridLocation.RED_PHYSICAL_BAY_4),

    LOGICAL_BAY_5(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_5, PhysicalGridLocation.RED_PHYSICAL_BAY_5),

    LOGICAL_BAY_6(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_6, PhysicalGridLocation.RED_PHYSICAL_BAY_6),

    LOGICAL_BAY_7(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_7, PhysicalGridLocation.RED_PHYSICAL_BAY_7),

    LOGICAL_BAY_8(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_8, PhysicalGridLocation.RED_PHYSICAL_BAY_8),

    LOGICAL_BAY_9(
        PhysicalGridLocation.BLUE_PHYSICAL_BAY_9, PhysicalGridLocation.RED_PHYSICAL_BAY_9);

    public final PhysicalGridLocation bluePhysical;
    public final PhysicalGridLocation redPhysical;

    private LogicalGridLocation(
        PhysicalGridLocation bluePhysical, PhysicalGridLocation redPhysical) {
      this.bluePhysical = bluePhysical;
      this.redPhysical = redPhysical;
    }
  }

  public enum DeadzoneBox {
    TEST_DEADZONE(
        new Translation2d(10, 10),
        new Translation2d(10, 5),
        new Translation2d(5, 10),
        new Translation2d(5, 5)),

    BLUE_COMMUNITY(new Translation2d(1.4, 5.3), new Translation2d(3.4, 0.0)),
    BLUE_COMMUNITY_PART_2(new Translation2d(2.9, 1.5), new Translation2d(5.0, 0.0)),

    BLUE_ENTRANCE_HUMAN_PLAYER_SIDE(new Translation2d(3.0, 5.3), new Translation2d(5.0, 4.2));

    private Translation2d topL;
    private Translation2d topR;
    private Translation2d backL;
    private Translation2d backR;

    private DeadzoneBox(
        Translation2d topL, Translation2d topR, Translation2d backL, Translation2d backR) {
      this.topL = topL;
      this.topR = topR;
      this.backL = backL;
      this.backR = backR;
    }

    private DeadzoneBox(Translation2d backL, Translation2d topR) {
      this.backL = backL;
      this.topR = topR;

      this.topL = new Translation2d(topR.getX(), backL.getY());
      this.backR = new Translation2d(backL.getX(), topR.getY());
    }

    public boolean insideBox(Translation2d point) {
      double x = point.getX();
      double y = point.getY();

      double x1 = backL.getX();
      double y1 = backL.getY();

      double x2 = topR.getX();
      double y2 = topR.getY();

      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/point/X", x);
      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/point/Y", y);

      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/backL/X", x1);
      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/backL/Y", y1);

      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/topR/X", x2);
      Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/topR/Y", y2);

      if (x > x1 && x < x2) {
        Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/in X", true);
      } else {
        Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/in X", false);
      }
      if (y < y1 && y > y2) {
        Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/in Y", true);
      } else {
        Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox/in Y", false);
      }

      if ((x > x1 && x < x2) && (y < y1 && y > y2)) {
        return true;
      }

      return false;
    }

    public void Log() {
      var logger = Logger.getInstance();
      logger.recordOutput(
          ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/topL",
          new Pose2d(topL, new Rotation2d()));
      logger.recordOutput(
          ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/topR",
          new Pose2d(topR, new Rotation2d()));
      logger.recordOutput(
          ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/backL",
          new Pose2d(backL, new Rotation2d()));
      logger.recordOutput(
          ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/backR",
          new Pose2d(backR, new Rotation2d()));

      ArrayList<State> states = new ArrayList<State>();
      states.add(createState(topL));
      states.add(createState(topR));
      states.add(createState(backR));
      states.add(createState(backL));
      states.add(createState(topL));

      Trajectory traj = new Trajectory(states);

      logger.recordOutput(ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/traj", traj);
    }

    private State createState(Translation2d loc) {
      return new State(0, 0, 0, new Pose2d(loc, new Rotation2d()), 0);
    }
  }
}
