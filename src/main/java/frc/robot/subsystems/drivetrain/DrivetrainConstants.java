package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;

public abstract class DrivetrainConstants {
  public int FRONT_LEFT_MODULE_DRIVE_MOTOR;
  public int FRONT_LEFT_MODULE_STEER_MOTOR;
  public int FRONT_LEFT_MODULE_STEER_ENCODER;
  public double FRONT_LEFT_MODULE_STEER_OFFSET;
  public boolean FRONT_LEFT_MODULE_INVERT_DRIVE;
  public boolean FRONT_LEFT_MODULE_INVERT_STEER;

  public int FRONT_RIGHT_MODULE_DRIVE_MOTOR;
  public int FRONT_RIGHT_MODULE_STEER_MOTOR;
  public int FRONT_RIGHT_MODULE_STEER_ENCODER;
  public double FRONT_RIGHT_MODULE_STEER_OFFSET;
  public boolean FRONT_RIGHT_MODULE_INVERT_DRIVE;
  public boolean FRONT_RIGHT_MODULE_INVERT_STEER;

  public int BACK_LEFT_MODULE_DRIVE_MOTOR;
  public int BACK_LEFT_MODULE_STEER_MOTOR;
  public int BACK_LEFT_MODULE_STEER_ENCODER;
  public double BACK_LEFT_MODULE_STEER_OFFSET;
  public boolean BACK_LEFT_MODULE_INVERT_DRIVE;
  public boolean BACK_LEFT_MODULE_INVERT_STEER;

  public int BACK_RIGHT_MODULE_DRIVE_MOTOR;
  public int BACK_RIGHT_MODULE_STEER_MOTOR;
  public int BACK_RIGHT_MODULE_STEER_ENCODER;
  public double BACK_RIGHT_MODULE_STEER_OFFSET;
  public boolean BACK_RIGHT_MODULE_INVERT_DRIVE;
  public boolean BACK_RIGHT_MODULE_INVERT_STEER;

  public int PIGEON_ID;
  public String PIGEON_CAN_BUS_NAME = "CANivore";

  // FIXME: update robot dimensions

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public double TRACKWIDTH_METERS;

  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public double WHEELBASE_METERS;

  public double ROBOT_WIDTH_WITH_BUMPERS;
  public double ROBOT_LENGTH_WITH_BUMPERS;

  /* The geometry and coordinate systems can be confusing. Refer to this document
  for a detailed explanation: https://docs.google
  .com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
  */
  public SwerveDriveKinematics KINEMATICS;

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
  public double MAX_VELOCITY_METERS_PER_SECOND;

  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  public double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  // TODO: find actual max angular acceleration
  public double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

  public double MAX_COAST_VELOCITY_METERS_PER_SECOND;

  public int TIMEOUT_MS = 30;

  public double AUTO_MAX_SPEED_METERS_PER_SECOND;
  public double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  public double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  public double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED;

  public double AUTO_TEST_MAX_SPEED_METERS_PER_SECOND;
  public double AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  public double AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  public double AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED;

  // FIXME: tune PID values for auto paths

  public double AUTO_DRIVE_P_CONTROLLER;
  public double AUTO_DRIVE_I_CONTROLLER;
  public double AUTO_DRIVE_D_CONTROLLER;

  public double AUTO_TURN_P_CONTROLLER;
  public double AUTO_TURN_I_CONTROLLER;
  public double AUTO_TURN_D_CONTROLLER;

  public double DEADBAND;

  protected void initializeRobotBase(
      double trackWidth, double wheelBase, double extraBumperWidth, double extraBumperLength) {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    TRACKWIDTH_METERS = trackWidth;
    ROBOT_WIDTH_WITH_BUMPERS = trackWidth + extraBumperWidth;

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    WHEELBASE_METERS = wheelBase;
    ROBOT_LENGTH_WITH_BUMPERS = wheelBase + extraBumperLength;

    /* The geometry and coordinate systems can be confusing. Refer to this document
    for a detailed explanation: https://docs.google
    .com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
    */
    KINEMATICS =
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
     * The maximum angular velocity of the robot in radians per second.
     *
     * <p>This is a measure of how fast the robot can rotate in place.
     */
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    // TODO: find actual max angular acceleration
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  public Drivetrain buildDriveTrain() {
    GyroIO gyro = new GyroIOPigeon2(PIGEON_ID, PIGEON_CAN_BUS_NAME);

    // FIXME: need to invert the back left drive motor and back right steer motor for 2023

    SwerveModule flModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                0,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET),
            0,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule frModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
            1,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule blModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                2,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
            2,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule brModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
            3,
            MAX_VELOCITY_METERS_PER_SECOND);

    return new Drivetrain(this, gyro, flModule, frModule, blModule, brModule);
  }
}
