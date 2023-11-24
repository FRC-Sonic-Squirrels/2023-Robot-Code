package frc.robot.subsystems.SimMechanism;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SimulatedMechanism extends SubsystemBase {
  private static SimulatedMechanism instance;

  private static final double FACING_UP = 90;
  private static final double FACING_RIGHT = 0;
  private static final double FACING_LEFT = 180;
  private static final double FACING_DOWN = 270;

  private static final double MECHANISM_WIDTH = 100;
  private static final double MECHANISM_HEIGHT = 100;

  private static final double ALLIANCE_WALL_TO_HIGH_POLE = 14.5669;
  private static final double ALLIANCE_WALL_TO_MID_POLE = 31.5;

  private static final double GRID_LENGTH = 54.25;
  private static final double ALLIANCE_WALL_TO_PRETRUSION_START = GRID_LENGTH - 16;
  private static final double PROTRUSION_LENGTH = 16;

  private static final double MID_POLE_HEIGHT = 34;
  private static final double HIGH_POLE_HEIGHT = 46;

  private static final double BUMPER_WIDTH = 1.25;
  private static final double ROBOT_WIDTH = 28 + BUMPER_WIDTH * 2;

  private static final double ELEVATOR_OFF_GROUND_HEIGHT = 9.375;
  private static final double ROBOT_TO_ELEVATOR_X = 4.5 + BUMPER_WIDTH;

  private static final double ROBOT_LEFT_ROOT_X = MECHANISM_WIDTH - GRID_LENGTH - ROBOT_WIDTH;
  private static final double ELEVATOR_ROOT_X = ROBOT_LEFT_ROOT_X + ROBOT_TO_ELEVATOR_X;

  private static final double ELEVATOR_ANGLE = 70;

  private double elevatorMinLengthInches = 0;
  private double stingerMinLengthInches = 24.2;

  // private double elevatorLength = elevatorMinLengthInches;
  // private double stingerlength = stingerMinLengthInches;

  private double elevatorLength;
  private double stingerlength;

  // TODO: get actual numbers of elevator and stinger
  private static final double MAIN_MECH_WIDTH = ROBOT_WIDTH;
  private static final double MAIN_MECH_HEIGHT = 50;

  Mechanism2d mainMech =
      new Mechanism2d(
          Units.inchesToMeters(MAIN_MECH_WIDTH), Units.inchesToMeters(MAIN_MECH_HEIGHT));

  MechanismRoot2d mainMechRoot =
      mainMech.getRoot(
          "mainMechRoot",
          Units.inchesToMeters(ROBOT_TO_ELEVATOR_X),
          Units.inchesToMeters(ELEVATOR_OFF_GROUND_HEIGHT));

  MechanismLigament2d elevatorLigament =
      mainMechRoot.append(new MechanismLigament2d("elevator", 1, ELEVATOR_ANGLE));

  MechanismLigament2d stingerLigament =
      elevatorLigament.append(new MechanismLigament2d("stinger", 10, -ELEVATOR_ANGLE));

  private SimulatedMechanism() {}

  public static SimulatedMechanism getInstance() {
    if (instance == null) {
      instance = new SimulatedMechanism();
      return instance;
    } else {
      return instance;
    }
  }

  public void setElevatorLengthInches(double lengthInches) {
    Logger.getInstance()
        .recordOutput("Elevator/lengthMeters set", elevatorMinLengthInches + lengthInches);
    elevatorLength = elevatorMinLengthInches + lengthInches;
    Logger.getInstance().recordOutput("Elevator/lengthMeters set after math", elevatorLength);
  }

  public void setStingerLengthInches(double lengthInches) {
    Logger.getInstance()
        .recordOutput("Elevator/stinger set", stingerMinLengthInches + lengthInches);
    stingerlength = stingerMinLengthInches + lengthInches;
  }

  @Override
  public void periodic() {

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages

    // Update elevator visualization with simulated position

    elevatorLigament.setLength(Units.inchesToMeters(elevatorLength));
    stingerLigament.setLength(Units.inchesToMeters(stingerlength));

    // var stingerLength = stingerSim.getPositionMeters();
    // var stingerInches = Units.metersToInches(stingerLength);

    // stingerMech2d.setLength(stingerInches);

    // double wristAngleRad = wristSim.getAngleRads();
    // double wristAngleDegrees = Units.radiansToDegrees(wristAngleRad);

    // double speed = wristOutput;

    // wristMech2d.setAngle(wristAngleDegrees);

    Pose3d elevatorSlider =
        new Pose3d(
            Units.inchesToMeters(
                (elevatorLength - elevatorMinLengthInches)
                    * (1.0 / 2.0)
                    * Math.cos(Math.toRadians(70))),
            0.0,
            Units.inchesToMeters(
                (elevatorLength - elevatorMinLengthInches)
                    * (1.0 / 2.0)
                    * Math.sin(Math.toRadians(70))),
            new Rotation3d(0, 0, 0));

    Pose3d stingerApparatus =
        new Pose3d(
            Units.inchesToMeters(
                (elevatorLength - elevatorMinLengthInches) * Math.cos(Math.toRadians(70))),
            0.0,
            Units.inchesToMeters(
                (elevatorLength - elevatorMinLengthInches) * Math.sin(Math.toRadians(70))),
            new Rotation3d(0, 0, 0));

    Pose3d stingerSlider =
        stingerApparatus.transformBy(
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters((stingerlength - stingerMinLengthInches) * 0.45),
                    0.0,
                    0.0),
                new Rotation3d(0, 0, 0)));

    Pose3d stingerIntake =
        stingerApparatus.transformBy(
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters((stingerlength - stingerMinLengthInches) * 0.9), 0.0, 0.0),
                new Rotation3d(0, 0, 0)));

    Pose3d stingerJaw;
    if (elevatorLength - elevatorMinLengthInches <= 5
        && stingerlength - stingerMinLengthInches <= 15) {
      stingerJaw =
          stingerIntake.transformBy(
              new Transform3d(
                  new Translation3d(
                      (elevatorLength - elevatorMinLengthInches) / 5 * -0.045,
                      0.0,
                      (elevatorLength - elevatorMinLengthInches) / 5 * 0.136),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians((elevatorLength - elevatorMinLengthInches) / 5 * 40),
                      0)));
    } else {
      stingerJaw =
          stingerIntake.transformBy(
              new Transform3d(
                  new Translation3d(-0.045, 0.0, 0.136),
                  new Rotation3d(0, Units.degreesToRadians(40), 0)));
    }

    Logger.getInstance().recordOutput("Mechanism/MainMechSimple", mainMech);
    Logger.getInstance()
        .recordOutput(
            "Mechanism/MainMech",
            elevatorSlider,
            stingerApparatus,
            stingerSlider,
            stingerIntake,
            stingerJaw);
  }
}
