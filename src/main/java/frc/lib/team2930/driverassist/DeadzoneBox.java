package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public enum DeadzoneBox {
  // TODO: add a flip for red option so making bounding boxes for red is less manual work
  TEST_DEADZONE(
      new Translation2d(10, 10),
      new Translation2d(10, 5),
      new Translation2d(5, 10),
      new Translation2d(5, 5)),

  BLUE_COMMUNITY(new Translation2d(1.4, 5.3), new Translation2d(3.4, 0.0)),

  BLUE_ENTRANCE_WALL_SIDE(new Translation2d(2.9, 1.5), new Translation2d(5.5, 0.0)),
  BLUE_ENTRANCE_HUMAN_PLAYER_SIDE(new Translation2d(3.0, 5.3), new Translation2d(5.5, 4.2)),

  BLUE_IN_FRONT_PAD(new Translation2d(5.5, 5.3), new Translation2d(6.5, 0.0)),

  BLUE_SKIP_CHECKPOINT(new Translation2d(1.4, 5.3), new Translation2d(2.9, 0.0)),

  // ----------------------RED----------------------------'

  RED_COMMUNITY(BLUE_COMMUNITY),

  RED_ENTRANCE_WALL_SIDE(BLUE_ENTRANCE_WALL_SIDE),
  RED_ENTRANCE_HUMAN_PLAYER_SIDE(BLUE_ENTRANCE_HUMAN_PLAYER_SIDE),

  RED_IN_FRONT_PAD(BLUE_IN_FRONT_PAD),

  RED_SKIP_CHECKPOINT(BLUE_SKIP_CHECKPOINT);

  private Translation2d topL;
  private Translation2d topR;
  private Translation2d backL;
  private Translation2d backR;

  private static final String ROOT_TABLE = GridPositionHandler.ROOT_TABLE;
  private static final double FIELD_WIDTH_METERS = GridPositionHandler.FIELD_WIDTH_METERS;

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

  private DeadzoneBox(DeadzoneBox blueToFlip) {
    this.backR =
        new Translation2d(blueToFlip.backL.getX(), FIELD_WIDTH_METERS - blueToFlip.backL.getY());
    this.topL =
        new Translation2d(blueToFlip.topL.getX(), FIELD_WIDTH_METERS - blueToFlip.topR.getY());

    this.topR = new Translation2d(topL.getX(), backR.getY());
    this.backL = new Translation2d(backR.getX(), topL.getY());
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
        ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/topL", new Pose2d(topL, new Rotation2d()));
    logger.recordOutput(
        ROOT_TABLE + "/DeadzoneBoxes/" + this.name() + "/topR", new Pose2d(topR, new Rotation2d()));
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
