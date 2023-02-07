package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import org.littletonrobotics.junction.Logger;

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

  RED_PHYSICAL_BAY_1(BLUE_PHYSICAL_BAY_9),
  RED_PHYSICAL_BAY_2(BLUE_PHYSICAL_BAY_8),
  RED_PHYSICAL_BAY_3(BLUE_PHYSICAL_BAY_7),
  RED_PHYSICAL_BAY_4(BLUE_PHYSICAL_BAY_6),
  RED_PHYSICAL_BAY_5(BLUE_PHYSICAL_BAY_5),
  RED_PHYSICAL_BAY_6(BLUE_PHYSICAL_BAY_4),
  RED_PHYSICAL_BAY_7(BLUE_PHYSICAL_BAY_3),
  RED_PHYSICAL_BAY_8(BLUE_PHYSICAL_BAY_2),
  RED_PHYSICAL_BAY_9(BLUE_PHYSICAL_BAY_1),

  ERROR_0_0(
      new PoseAndHeading(
          new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)),
      new PoseAndHeading(
          new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)));

  public final PoseAndHeading lineup;
  public final PoseAndHeading score;

  private static final String ROOT_TABLE = GridPositionHandler.ROOT_TABLE;
  private static final double FIELD_WIDTH_METERS = GridPositionHandler.FIELD_WIDTH_METERS;

  private PhysicalGridLocation(PoseAndHeading lineup, PoseAndHeading score) {
    this.lineup = lineup;
    this.score = score;
  }

  private PhysicalGridLocation(PhysicalGridLocation blueToFlip) {
    var blueLineup = blueToFlip.lineup;

    this.lineup =
        new PoseAndHeading(
            new Pose2d(
                blueLineup.pose.getX(),
                FIELD_WIDTH_METERS - blueLineup.pose.getY(),
                blueLineup.pose.getRotation()),
            blueLineup.heading);

    var blueScore = blueToFlip.score;

    this.score =
        new PoseAndHeading(
            new Pose2d(
                blueScore.pose.getX(),
                FIELD_WIDTH_METERS - blueScore.pose.getY(),
                blueScore.pose.getRotation()),
            blueScore.heading);
  }

  public void log() {
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/PHYSICAL_BAY/" + this.name() + "/lineup", this.lineup.pose);
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/PHYSICAL_BAY/" + this.name() + "/score", this.score.pose);
  }
}
