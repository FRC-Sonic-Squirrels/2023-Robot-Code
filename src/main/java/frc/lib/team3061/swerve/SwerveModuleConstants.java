/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.lib.team3061.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public final class SwerveModuleConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SwerveModuleConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  /* MK4i L2 */
  public static final double MK4I_L2_WHEEL_DIAMETER_METERS = 0.10033;
  public static final double MK4I_L2_WHEEL_CIRCUMFERENCE_THEORETICAL =
      MK4I_L2_WHEEL_DIAMETER_METERS * Math.PI;
  public static final double MK4I_L2_WHEEL_CIRCUMFERENCE = 0.31115; // measured 2/24/2023 90% tread

  public static final double MK4I_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final boolean MK4I_L2_DRIVE_MOTOR_INVERTED = true;
  public static final double MK4I_L2_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
  public static final boolean MK4I_L2_ANGLE_MOTOR_INVERTED = true;
  public static final boolean MK4I_L2_CAN_CODER_INVERTED = false;

  /* MK4 L2 */
  public static final double MK4_L2_WHEEL_DIAMETER_METERS = 0.10033;
  public static final double MK4_L2_WHEEL_CIRCUMFERENCE = MK4_L2_WHEEL_DIAMETER_METERS * Math.PI;
  public static final double MK4_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final boolean MK4_L2_DRIVE_MOTOR_INVERTED = true;
  public static final double MK4_L2_ANGLE_GEAR_RATIO = 1 / ((15.0 / 32.0) * (10.0 / 60.0));
  public static final boolean MK4_L2_ANGLE_MOTOR_INVERTED = false;
  public static final boolean MK4_L2_CAN_CODER_INVERTED = false;

  // FIXME: assign these constants to the appropriate swerve module variant
  public static final double WHEEL_DIAMETER_METERS = MK4I_L2_WHEEL_DIAMETER_METERS;
  // MK4I_L2_WHEEL_CIRCUMFERENCE;
  public static final double WHEEL_CIRCUMFERENCE =
      0.308; // 0.3175; // measured 2/25/2023 at practice field
  public static final double DRIVE_GEAR_RATIO = MK4I_L2_DRIVE_GEAR_RATIO;
  public static final boolean DRIVE_MOTOR_INVERTED = MK4I_L2_DRIVE_MOTOR_INVERTED;
  public static final double ANGLE_GEAR_RATIO = MK4I_L2_ANGLE_GEAR_RATIO;
  public static final boolean ANGLE_MOTOR_INVERTED = MK4I_L2_ANGLE_MOTOR_INVERTED;
  public static final boolean CAN_CODER_INVERTED = MK4I_L2_CAN_CODER_INVERTED;

  public static final double OPEN_LOOP_RAMP = 0.25;
  public static final double CLOSED_LOOP_RAMP = 0.0;

  /* Swerve Current Limiting */
  public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
  public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
  public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

  public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
  public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
  public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

  /* PID SLOT INDEX */
  public static final int SLOT_INDEX = 0;

  // FIXME: tune PID values for the angle and drive motors

  /* Angle Motor PID Values */
  public static final double ANGLE_KP = 0.6;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 12.0;
  public static final double ANGLE_KF = 0.0;

  /* Drive Motor PID Values */
  public static final double DRIVE_KP = 0.10;
  public static final double DRIVE_KI = 0.0;
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KF = 0.0;

  // FIXME: characterize the drivetrain and update these constants

  /* Drive Motor Characterization Values */
  // divide by 12 to convert from volts to percent output for CTRE
  public static final double DRIVE_KS = (0.6 / 12); // 0.628 for heavy, 0.53114 light
  public static final double DRIVE_KV = (2.344 / 12); // 2.344 for heavy, 2.3423 light
  public static final double DRIVE_KA = (0.35 / 12); // 0.435 for heavy, 0.12817 light

  /* Simulated Angle Motor PID Values */
  public static final double SIM_ANGLE_KP = 12.0;
  public static final double SIM_ANGLE_KI = 0.0;
  public static final double SIM_ANGLE_KD = 0.0;
  public static final double SIM_ANGLE_KF = 0.0;

  /* Simulated Drive Motor PID Values */
  public static final double SIM_DRIVE_KP = 0.5;
  public static final double SIM_DRIVE_KI = 0.0;
  public static final double SIM_DRIVE_KD = 0.0;
  public static final double SIM_DRIVE_KF = 0.0;

  /* Simulated Drive Motor Characterization Values */
  public static final double SIM_DRIVE_KS = 0.116970;
  public static final double SIM_DRIVE_KV = 0.133240;
  public static final double SIM_DRIVE_KA = 0.0;

  /* Neutral Modes */
  public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
}
