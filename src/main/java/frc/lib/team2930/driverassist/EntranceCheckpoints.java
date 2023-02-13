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
          new Pose2d(2.5, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(3.85, 0.7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.5, 0.7, Rotation2d.fromDegrees(180)),
          Rotation2d.fromDegrees(180))), // x:2.17

  BLUE_HUMAN_PLAYER(
      new PoseAndHeading(
          new Pose2d(2.5, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(3.85, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)),
      new PoseAndHeading(
          new Pose2d(5.5, 4.65, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180))),

  // -----------------------------RED-----------------------

  RED_WALL(BLUE_WALL),

  RED_HUMAN_PLAYER(BLUE_HUMAN_PLAYER),

  ERROR();

  public final PoseAndHeading checkPointInside;
  public final PoseAndHeading checkPointMiddle;
  public final PoseAndHeading checkPointOutside;

  private static final String ROOT_TABLE = GridPositionHandler.ROOT_TABLE;
  private static final double FIELD_WIDTH_METERS = GridPositionHandler.FIELD_WIDTH_METERS;

  private PoseAndHeading[] orderOutsideIn;

  private EntranceCheckpoints(
      PoseAndHeading inside, PoseAndHeading middle, PoseAndHeading outside) {
    this.checkPointInside = inside;
    this.checkPointMiddle = middle;
    this.checkPointOutside = outside;

    orderOutsideIn = new PoseAndHeading[3];
    orderOutsideIn[0] = outside;
    orderOutsideIn[1] = middle;
    orderOutsideIn[2] = inside;
  }

  private EntranceCheckpoints(EntranceCheckpoints blueToFlip) {
    var checkPointsOutSideIn = blueToFlip.getOrderOutsideIn();

    PoseAndHeading[] redSideCheckpoints = new PoseAndHeading[3];

    for (int i = 0; i < checkPointsOutSideIn.length; i++) {
      PoseAndHeading original = checkPointsOutSideIn[i];

      Pose2d alteredPose =
          new Pose2d(
              original.pose.getX(),
              FIELD_WIDTH_METERS - original.pose.getY(),
              original.pose.getRotation());

      redSideCheckpoints[i] = new PoseAndHeading(alteredPose, original.heading);

      this.orderOutsideIn = redSideCheckpoints;
    }

    checkPointOutside = redSideCheckpoints[0];
    checkPointMiddle = redSideCheckpoints[1];
    checkPointInside = redSideCheckpoints[2];
  }

  private EntranceCheckpoints() {
    this.checkPointInside = new PoseAndHeading();
    this.checkPointMiddle = new PoseAndHeading();
    this.checkPointOutside = new PoseAndHeading();
  }

  public PoseAndHeading[] getOrderOutsideIn() {
    return orderOutsideIn;
  }

  public static PathPoint toPathPoint(PoseAndHeading selectedCheckPoint) {
    return new PathPoint(
        selectedCheckPoint.pose.getTranslation(),
        selectedCheckPoint.heading,
        selectedCheckPoint.pose.getRotation());
  }

  public void log() {
    Pose2d insidePose =
        new Pose2d(checkPointInside.pose.getTranslation(), checkPointInside.heading);
    Pose2d middlePose =
        new Pose2d(checkPointMiddle.pose.getTranslation(), checkPointMiddle.heading);
    Pose2d outsidePose =
        new Pose2d(checkPointOutside.pose.getTranslation(), checkPointOutside.heading);

    Logger.getInstance()
        .recordOutput(
            "DriverAssist/GridPosition/entrance " + this.name() + "/insideCheckpoint", insidePose);
    Logger.getInstance()
        .recordOutput(
            "DriverAssist/GridPosition/entrance " + this.name() + "/middleCheckpoint", middlePose);
    Logger.getInstance()
        .recordOutput(
            "DriverAssist/GridPosition/entrance " + this.name() + "/outsideCheckpoint",
            outsidePose);
  }
}
