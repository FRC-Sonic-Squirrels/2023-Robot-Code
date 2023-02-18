package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.util.Units;
import frc.lib.team3061.swerve.SwerveModuleConstants;

public class DrivetrainConstants2022 extends DrivetrainConstants {
    public DrivetrainConstants2022() {
        FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        FRONT_LEFT_MODULE_STEER_MOTOR = 11;
        FRONT_LEFT_MODULE_STEER_ENCODER = 21;
        FRONT_LEFT_MODULE_STEER_OFFSET = 320.2;

        FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
        FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
        FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
        FRONT_RIGHT_MODULE_STEER_OFFSET = 93.3;

        BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
        BACK_LEFT_MODULE_STEER_MOTOR = 14;
        BACK_LEFT_MODULE_STEER_ENCODER = 24;
        BACK_LEFT_MODULE_STEER_OFFSET = 282.4;

        BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
        BACK_RIGHT_MODULE_STEER_MOTOR = 13;
        BACK_RIGHT_MODULE_STEER_ENCODER = 23;
        BACK_RIGHT_MODULE_STEER_OFFSET = 329.2;

        PIGEON_ID = 15;

        initializeRobotBase(Units.inchesToMeters(25), Units.inchesToMeters(24), 10, 12);

        MAX_VELOCITY_METERS_PER_SECOND =
                6380.0
                        / 60.0
                        / SwerveModuleConstants.DRIVE_GEAR_RATIO
                        * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

        MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

        AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
        AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
        AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

        AUTO_TEST_MAX_SPEED_METERS_PER_SECOND = 0.2;
        AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;
        AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0.2 * Math.PI;
        AUTO_TEST_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 0.2 * Math.PI;

        // FIXME: tune PID values for auto paths

        AUTO_DRIVE_P_CONTROLLER = 2.2941;
        AUTO_DRIVE_I_CONTROLLER = 0.0;
        AUTO_DRIVE_D_CONTROLLER = 0.0;

        AUTO_TURN_P_CONTROLLER = 4.9;
        AUTO_TURN_I_CONTROLLER = 0.0;
        AUTO_TURN_D_CONTROLLER = 0.0;

        DEADBAND = 0.1;
    }
}
