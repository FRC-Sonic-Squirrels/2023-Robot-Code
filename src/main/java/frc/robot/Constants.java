// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = true;

  public static final String CAN_BUS_NAME = "CANivore";

  // FIXME: specify the name of the camera used for detecting AprilTags

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // FIXME: FIXMEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
  // FIXME: make sure these are the correct names
  public static final String LEFT_CAMERA_NAME = "LeftCamera";
  public static final String RIGHT_CAMERA_NAME = "RightCamera";

  private static final RobotType ROBOT = RobotType.ROBOT_2023_COMPBOT;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023_PRESEASON;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023_PRESEASON:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_2023_COMPBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_2023_PRESEASON,
    ROBOT_2023_COMPBOT,
    ROBOT_SIMBOT
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static final class CANIVORE_canId {
    // CANIvore Can Ids
    public static final String name = "CANivore";

    // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will
    // create problems if you already have a device using CAN id 0 on the CAN bus.
    public static final int DoNotUse_canId0 = 0;

    // NOTE: Swerve uses CANids: 1,2,3,4,11,12,13,14,21,22,23,24
    public static final int CANID15_pigeon_imu = 15;
  }

  public static final class CanId {
    // NON-CANivore Can Ids
    public static final String name = "";

    // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will
    // create problems if you already have a device using CAN id 0 on the CAN bus.
    public static final int DoNotUse_canId0 = 0;

    public static final int CANID5_STINGER_TALON = 5;
    public static final int CANID6_INTAKE_TALON = 6;
    public static final int CANID9_ELEVATOR_LEAD_TALON = 9;
    public static final int CANID10_ELEVATOR_FOLLOW_TALON = 10;
  }

  public static final class pneumatics {
    public static final int channel_0 = 0;
    public static final int channel_1 = 1;
    public static final int channel_2 = 2;
    public static final int channel_3 = 3;
    public static final int channel_4 = 4;
    public static final int channel_5 = 5;
    public static final int channel_6 = 6;
    public static final int channel_7 = 7;
    public static final int channel_8 = 8;
    public static final int channel_9 = 9;
    public static final int channel_10 = 10;
    public static final int channel_11 = 11;
    public static final int channel_12 = 12;
    public static final int channel_13 = 13;
    public static final int channel_14_intake = 14;
    public static final int channel_15_friction_brake = 15;
  }

  public static class PWMPorts {
    public static final int kBlinkin = 0;
  }

  // TODO: determine whether elevator deserves it's own constants file
  public static class Elevator {

    public static final double MAX_HEIGHT_INCHES = 48.9; // 49.1 max physical

    // https://ss2930.sharepoint.com/:x:/r/sites/Programming/_layouts/15/Doc.aspx?sourcedoc=%7B318D8C0F-AC95-43F3-B4DB-0964BE9A2FD1%7D&file=elevator%202023%20howdybots%20version.xlsx&action=default&mobileredirect=true
    public static final double F_CONTROLLER = 0.0; // jvn velocity 0.024427;
    public static final double P_CONTROLLER = 0.12; // jvn velocity 0.098864;
    public static final double I_CONTROLLER = 0.0;
    public static final double D_CONTROLLER = 0.0;

    // Arbitrary feed forward voltage is to offset gravity.
    // Holding voltage is between 0.25-0.45 Volts. Convert to percent output.
    public static final double ARBITRARY_FEED_FORWARD = 0.18 / 12.0;
    public static final double CRUISE_VELOCITY_INCHES_PER_SEC = 45.0;
    public static final double DESIRED_TIME_TO_SPEED = 0.25;
  }

  public static final class Stinger {

    public static final double MAX_EXTENSION_INCHES = 25.0; // real life: 26.0, max sensor is 25.6

    // TODO: tune PIDF for stinger
    // https://ss2930.sharepoint.com/:x:/r/sites/Programming/_layouts/15/Doc.aspx?sourcedoc=%7B318D8C0F-AC95-43F3-B4DB-0964BE9A2FD1%7D&file=elevator%202023%20howdybots%20version.xlsx&action=default&mobileredirect=true
    public static final double STINGER_FEEDFORWARD = 0.0;
    public static final double STINGER_KP = 0.12;
    public static final double STINGER_KI = 0.0;
    public static final double STINGER_KD = 0.0;

    public static final double CRUISE_VELOCITY_INCHES_PER_SEC = 40;
    public static final double DESIRED_TIME_TO_SPEED = 0.25;

    // Arbitrary feed forward is in percent output (volts/12.0)
    public static final double ARBITRARY_FEED_FORWARD = 0.0 / 12.0;
  }

  // the depth and height of field nodes compared to a robot right in front of them
  public static final class NODE_DISTANCES {

    // all these measurements are in INCHES
    public static final double EXTENSION_LOW = 7; // half the depth of the hybrid node
    public static final double EXTENSION_MID_CUBE = 16.3;
    public static final double EXTENSION_MID_CONE = 12.5;
    public static final double EXTENSION_HIGH_CONE = Stinger.MAX_EXTENSION_INCHES;
    public static final double EXTENSION_HIGH_CUBE = Stinger.MAX_EXTENSION_INCHES;

    // these measurements are 2 inches higher than the actual node heights so the items can make it
    // over
    public static final double HEIGHT_LOW = 11;
    public static final double HEIGHT_MID_CUBE = 30;
    public static final double HEIGHT_MID_CONE = 37;
    public static final double HEIGHT_HIGH_CUBE = 45.5;
    public static final double HEIGHT_HIGH_CONE = Elevator.MAX_HEIGHT_INCHES;

    public static final double STOW_HEIGHT = 6;
    public static final double STOW_EXTENSION = 0;
  }

  public static class Elevator2022 {

    public static final double MAX_HEIGHT_INCHES = 25.0;

    // https://ss2930.sharepoint.com/:x:/r/sites/Programming/_layouts/15/Doc.aspx?sourcedoc=%7B318D8C0F-AC95-43F3-B4DB-0964BE9A2FD1%7D&file=elevator%202023%20howdybots%20version.xlsx&action=default&mobileredirect=true
    public static final double F_CONTROLLER = 1.0;
    public static final double P_CONTROLLER = 0.48;
    public static final double I_CONTROLLER = 0.0;
    public static final double D_CONTROLLER = 0.0;

    public static final double CRUISE_VELOCITY_INCHES_PER_SEC = 40;
    public static final double DESIRED_TIME_TO_SPEED = 0.1;
  }
}
