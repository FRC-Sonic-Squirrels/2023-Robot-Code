package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // to get field layout call this:
  //    AprilTagFields.k2023ChargedUp.loadAprilTagFieldLayout();

  // Transform from the robot to each camera
  public static final Transform3d LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.51), Units.inchesToMeters(10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(30.0)));

  public static final Transform3d RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.51), Units.inchesToMeters(-10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-30.0)));

  public static final Transform3d BACK_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-2.7), Units.inchesToMeters(0.0), Units.inchesToMeters(33.42)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(10.0), Math.toRadians(180.0)));

  // TODO: probably want to increase this a little
  public static final double MAXIMUM_AMBIGUITY = 0.08;

  public static final double MAX_VALID_DISTANCE_AWAY_METERS = 3.0;
}
