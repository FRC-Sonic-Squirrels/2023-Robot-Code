package frc.robot.subsystems.SimMechanism;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SimulatedMechanism extends SubsystemBase {
  private static SimulatedMechanism instance;

  private static final double m_armReduction = 10;
  private static final double m_armMass = 8.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(10);

  SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength,
          Units.degreesToRadians(-20),
          Units.degreesToRadians(120),
          m_armMass,
          false);

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

  private final Mechanism2d m_mech2d = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot("Elevator Root", ELEVATOR_ROOT_X, ELEVATOR_OFF_GROUND_HEIGHT);
  private final MechanismLigament2d elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", 1, ELEVATOR_ANGLE, 10, new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d stingerMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Stinger",
              Units.metersToInches(1),
              -ELEVATOR_ANGLE,
              10,
              new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d wristMech2d =
      stingerMech2d.append(new MechanismLigament2d("Wrist", 1, 0, 10, new Color8Bit(Color.kRed)));

  private final MechanismRoot2d gridRoot = m_mech2d.getRoot("grid root", MECHANISM_WIDTH, 0);
  private final MechanismLigament2d gridFloor =
      gridRoot.append(
          new MechanismLigament2d(
              "grid floor", GRID_LENGTH, FACING_LEFT, 10, new Color8Bit(Color.kBlack)));

  private final MechanismRoot2d gridHighPoleRoot =
      m_mech2d.getRoot("grid high pole root", MECHANISM_WIDTH - ALLIANCE_WALL_TO_HIGH_POLE, 0);
  private final MechanismLigament2d gridHighPole =
      gridHighPoleRoot.append(
          new MechanismLigament2d(
              "grid high pole", HIGH_POLE_HEIGHT, FACING_UP, 10, new Color8Bit(Color.kGreen)));

  private final MechanismRoot2d gridMidPoleRoot =
      m_mech2d.getRoot("grid mid pole root", MECHANISM_WIDTH - ALLIANCE_WALL_TO_MID_POLE, 0);
  private final MechanismLigament2d gridMidPole =
      gridMidPoleRoot.append(
          new MechanismLigament2d(
              "grid mid pole", MID_POLE_HEIGHT, FACING_UP, 10, new Color8Bit(Color.kGreen)));

  private final MechanismRoot2d gridProtrusionStartRoot =
      m_mech2d.getRoot(
          "grid start of protrusion root",
          MECHANISM_WIDTH - ALLIANCE_WALL_TO_PRETRUSION_START,
          1.25);
  private final MechanismLigament2d gridProtrusion =
      gridProtrusionStartRoot.append(
          new MechanismLigament2d(
              "protrusion length",
              PROTRUSION_LENGTH,
              FACING_LEFT,
              15,
              new Color8Bit(Color.kDarkRed)));

  private final MechanismRoot2d robotLeftRoot =
      m_mech2d.getRoot("robot left", ROBOT_LEFT_ROOT_X, 0);

  private final MechanismLigament2d robotBed =
      robotLeftRoot.append(
          new MechanismLigament2d(
              "robot bed", ROBOT_WIDTH, FACING_RIGHT, 30, new Color8Bit(Color.kOrange)));

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

  // ------------
  // TODO: get correct measurements
  Mechanism2d fieldMech =
      new Mechanism2d(
          Units.inchesToMeters(GRID_LENGTH + ROBOT_WIDTH + 2),
          Units.inchesToMeters(HIGH_POLE_HEIGHT + 2));

  MechanismRoot2d fieldGridRoot =
      fieldMech.getRoot(
          "fieldGridRoot",
          Units.inchesToMeters(GRID_LENGTH + ROBOT_WIDTH),
          Units.inchesToMeters(0.0));

  MechanismRoot2d highPoleRoot =
      fieldMech.getRoot(
          "highPoleRoot",
          Units.inchesToMeters(GRID_LENGTH + ROBOT_WIDTH - ALLIANCE_WALL_TO_HIGH_POLE),
          0.0);

  MechanismLigament2d highPoleLigament =
      highPoleRoot.append(
          new MechanismLigament2d(
              "highPoleLigament", Units.inchesToMeters(HIGH_POLE_HEIGHT), FACING_UP));

  MechanismRoot2d midPoleRoot =
      fieldMech.getRoot(
          "midPoleRoot",
          Units.inchesToMeters(GRID_LENGTH + ROBOT_WIDTH - ALLIANCE_WALL_TO_MID_POLE),
          0.0);

  MechanismLigament2d midPoleLigament =
      midPoleRoot.append(
          new MechanismLigament2d(
              "midPoleLigament", Units.inchesToMeters(MID_POLE_HEIGHT), FACING_UP));

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
        .recordOutput("elevator/lengthMeters set", elevatorMinLengthInches + lengthInches);
    elevatorLength = elevatorMinLengthInches + lengthInches;
    Logger.getInstance().recordOutput("elevator/lengthMeters set after math", elevatorLength);
  }

  public void setStingerLengthInches(double lengthInches) {
    Logger.getInstance()
        .recordOutput("elevator/stinger set", stingerMinLengthInches + lengthInches);
    stingerlength = stingerMinLengthInches + lengthInches;
  }

  @Override
  public void periodic() {

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages

    // Update elevator visualization with simulated position
    Logger.getInstance().recordOutput("elevator/periodic mech length", elevatorLength);
    elevatorMech2d.setLength(elevatorLength);

    stingerMech2d.setLength(stingerlength);

    elevatorLigament.setLength(Units.inchesToMeters(elevatorLength));
    stingerLigament.setLength(Units.inchesToMeters(stingerlength));

    // var stingerLength = stingerSim.getPositionMeters();
    // var stingerInches = Units.metersToInches(stingerLength);

    // stingerMech2d.setLength(stingerInches);

    // double wristAngleRad = wristSim.getAngleRads();
    // double wristAngleDegrees = Units.radiansToDegrees(wristAngleRad);

    // double speed = wristOutput;

    // wristMech2d.setAngle(wristAngleDegrees);

    Logger.getInstance().recordOutput("SimMech", m_mech2d);

    Logger.getInstance().recordOutput("MainMech", mainMech);

    Logger.getInstance().recordOutput("FieldMech", fieldMech);
  }
}
