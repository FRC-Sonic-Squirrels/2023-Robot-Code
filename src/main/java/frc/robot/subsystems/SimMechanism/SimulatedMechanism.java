package frc.robot.subsystems.SimMechanism;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

  private static final double kElevatorKp = 5.0;
  private static final double kElevatorGearing = 10.0;
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 4.0; // kg

  private static final double kMinElevatorHeight = Units.inchesToMeters(20);
  private static final double kMaxElevatorHeight = Units.inchesToMeters(60);

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;

  private final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);

  // ElevatorSim elevatorSim =
  //     new ElevatorSim(
  //         elevatorGearbox,
  //         kElevatorGearing,
  //         kCarriageMass,
  //         kElevatorDrumRadius,
  //         kMinElevatorHeight,
  //         kMaxElevatorHeight,
  //         // true,
  //         false,
  //         VecBuilder.fill(0.01));

  ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeight,
          kMaxElevatorHeight,
          // true,
          false);

  // LinearSystem<N2, N1, N1> plant;

  // LinearSystemSim<N2, N1, N1> stingerSim = new LinearSystemSim<N2, N1, N1>(plant);

  ElevatorSim stingerSim =
      new ElevatorSim(
          elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          Units.inchesToMeters(10),
          Units.inchesToMeters(30),
          false);

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

  private final Mechanism2d m_mech2d = new Mechanism2d(100, 100);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              Units.metersToInches(elevatorSim.getPositionMeters()),
              45,
              10,
              new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d stingerMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Stinger",
              Units.metersToInches(stingerSim.getPositionMeters()),
              -45,
              10,
              new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d wristMech2d =
      stingerMech2d.append(
          new MechanismLigament2d(
              "Wrist", Units.metersToInches(m_armLength), 0, 10, new Color8Bit(Color.kRed)));

  private double desiredOutput;
  private double stingerOutput;
  private double wristOutput = 0.0;

  private SimulatedMechanism() {}

  public static SimulatedMechanism getInstance() {
    if (instance == null) {
      return new SimulatedMechanism();
    } else {
      return instance;
    }
  }

  @Override
  public void periodic() {
    // elevatorSim.setInput(desiredOutput * RobotController.getBatteryVoltage());
    elevatorSim.setInput(desiredOutput);

    Logger.getInstance().recordOutput("elevator/simDesiredOutput", desiredOutput);
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    stingerSim.setInput(stingerOutput);

    stingerSim.update(0.020);

    wristSim.setInput(wristOutput);

    wristSim.update(0.020);

    // stingerSim.update(0.0200);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    elevatorMech2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
    var stingerLength = stingerSim.getPositionMeters();
    var stingerInches = Units.metersToInches(stingerLength);

    stingerMech2d.setLength(stingerInches);
    // stingerMech2d.setAngle(new Rotation2d());

    // stingerMech2d.setLength(10);

    // wristOutput += 1

    double wristAngleRad = wristSim.getAngleRads();
    double wristAngleDegrees = Units.radiansToDegrees(wristAngleRad);

    double speed = wristOutput;

    wristMech2d.setAngle(wristAngleDegrees);

    Logger.getInstance().recordOutput("SimMech", m_mech2d);
  }

  public void setElevatorOutputVolts(double output) {
    desiredOutput = output;
  }

  public void setStingerOutputVolts(double output) {
    stingerOutput = output;
  }

  public void setWristOutputVolts(double output) {
    wristOutput = output;
  }

  public double getElevatorPositionInches() {
    Logger.getInstance()
        .recordOutput(
            "elevator/subtractedHeight",
            Units.metersToInches(elevatorSim.getPositionMeters())
                - Units.metersToInches(kMinElevatorHeight));

    return (Units.metersToInches(elevatorSim.getPositionMeters())
        - Units.metersToInches(kMinElevatorHeight));
  }

  public double getStingerPositionInches() {
    return Units.metersToInches(stingerSim.getPositionMeters());
  }

  public double getWristPositionRad() {
    return wristSim.getAngleRads();
  }
}
