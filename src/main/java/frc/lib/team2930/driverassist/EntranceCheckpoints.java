package frc.lib.team2930.driverassist;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import org.littletonrobotics.junction.Logger;

public enum EntranceCheckpoints {
  // TODO: Add a to pathpoint function(add this to the other physical grid locations as well)
  BLUE_WALL(
      new PoseAndHeading(
          new Pose2d(5.5, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.2, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.0, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(4.8, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(3.85, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(2.5, 0.7, Rotation2d.fromDegrees(180)),
          Rotation2d.fromDegrees(180))), // x:2.17

  BLUE_HUMAN_PLAYER(
      new PoseAndHeading(
          new Pose2d(5.5, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.2, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.0, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(4.8, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(3.85, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(2.5, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180))),

  // -----------------------------RED-----------------------

  RED_WALL(BLUE_WALL),

  RED_HUMAN_PLAYER(BLUE_HUMAN_PLAYER),

  ERROR();

  private static final String ROOT_TABLE = GridPositionHandler.ROOT_TABLE;
  private static final double FIELD_WIDTH_METERS = GridPositionHandler.FIELD_WIDTH_METERS;

  private final PoseAndHeading[] orderOutsideIn;

  // private EntranceCheckpoints(
  //     PoseAndHeading inside, PoseAndHeading middle, PoseAndHeading outside) {
  //   this.checkPointInside = inside;
  //   this.checkPointMiddle = middle;
  //   this.checkPointOutside = outside;

  //   orderOutsideIn = new PoseAndHeading[3];
  //   orderOutsideIn[0] = outside;
  //   orderOutsideIn[1] = middle;
  //   orderOutsideIn[2] = inside;
  // }

  private EntranceCheckpoints(PoseAndHeading... orderOutsideIn) {
    this.orderOutsideIn = orderOutsideIn;
  }

  private EntranceCheckpoints(EntranceCheckpoints blueToFlip) {
    var blueCheckpoints = blueToFlip.orderOutsideIn;

    this.orderOutsideIn = new PoseAndHeading[blueCheckpoints.length];

    for (int i = 0; i < blueCheckpoints.length; i++) {
      var blueCheckpoint = blueCheckpoints[i];
      var bluePose = blueCheckpoint.pose;
      var blueHeading = blueCheckpoint.heading;

      var redCheckpoint =
          new PoseAndHeading(
              new Pose2d(
                  bluePose.getX(), FIELD_WIDTH_METERS - bluePose.getY(), bluePose.getRotation()),
              blueHeading);

      this.orderOutsideIn[i] = redCheckpoint;
      // PoseAndHeading redCheckpoint = new PoseAndHeading(new Pose2d(, y, rotation), heading)
    }
  }

  public PoseAndHeading[] getOrderOutsideIn() {
    return orderOutsideIn;
  }

  public static PathPoint toPathPoint(PoseAndHeading selectedPoint) {
    return new PathPoint(
        selectedPoint.pose.getTranslation(),
        selectedPoint.heading,
        selectedPoint.pose.getRotation());
  }

  public void log(String ROOT) {
    // Pose2d insidePose =
    //     new Pose2d(checkPointInside.pose.getTranslation(), checkPointInside.heading);
    // Pose2d middlePose =
    //     new Pose2d(checkPointMiddle.pose.getTranslation(), checkPointMiddle.heading);
    // Pose2d outsidePose =
    //     new Pose2d(checkPointOutside.pose.getTranslation(), checkPointOutside.heading);

    // Logger.getInstance()
    //     .recordOutput(
    //         ROOT + "/EntranceCheckpoints/" + this.name() + "/insideCheckpoint", insidePose);
    // Logger.getInstance()
    //     .recordOutput(
    //         ROOT + "/EntranceCheckpoints/" + this.name() + "/middleCheckpoint", middlePose);
    // Logger.getInstance()
    //     .recordOutput(
    //         ROOT + "/EntranceCheckpoints/" + this.name() + "/outsideCheckpoint", outsidePose);

    for (int i = 0; i < orderOutsideIn.length; i++) {
      Logger.getInstance()
          .recordOutput(
              ROOT + "/EntranceCheckpoints/" + this.name() + "/" + i, orderOutsideIn[i].pose);
    }
  }
}
