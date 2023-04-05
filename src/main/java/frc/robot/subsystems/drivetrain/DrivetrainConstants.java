package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.swerve.SwerveModuleConstants;
import java.util.HashMap;

public final class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // 0
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 221.4;

  // 1
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 190.4;

  // Module 2
  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = 179.2; // 240.7; // 14.2;

  // module 3
  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 311.9; // 250.0; // 201.9; // 201.7;

  public static final int PIGEON_ID = 15;
  public static final String PIGEON_CAN_BUS_NAME = "";

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double TRACKWIDTH_METERS = Units.inchesToMeters(23);

  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double WHEELBASE_METERS = Units.inchesToMeters(25);

  public static final double ROBOT_WIDTH_WITH_BUMPERS = Units.inchesToMeters(34);
  public static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(36);

  /* The geometry and coordinate systems can be confusing. Refer to this document
  for a detailed explanation: https://docs.google.com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
  */
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));

  /**
   * The formula for calculating the theoretical maximum velocity is: <Motor free speed RPM> / 60 *
   * <Drive reduction> * <Wheel diameter meters> * pi By default this value is setup for a Mk3
   * standard module using Falcon500s to drive.
   */

  // FIXME: determine maximum velocities empirically

  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0
          / 60.0
          / SwerveModuleConstants.DRIVE_GEAR_RATIO
          * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

  // TODO: find actual max angular acceleration
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  public static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  public static final int TIMEOUT_MS = 30;

  public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.6;
  public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
  public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
  public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

  public static final double AUTO_TEST_MAX_SPEED_METERS_PER_SECOND = 0.2;
  public static final double AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;
  public static final double AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0.2 * Math.PI;
  public static final double AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 0.2 * Math.PI;

  // FIXME: tune PID values for auto paths

  public static final double AUTO_DRIVE_P_CONTROLLER = 2.2941;
  public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;

  public static final double AUTO_TURN_P_CONTROLLER = 4.9;
  public static final double AUTO_TURN_I_CONTROLLER = 0.0;
  public static final double AUTO_TURN_D_CONTROLLER = 0.0;

  public static final double DEADBAND = 0.1;

  public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
}
