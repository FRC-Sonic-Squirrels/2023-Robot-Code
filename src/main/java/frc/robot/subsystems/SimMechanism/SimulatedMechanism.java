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

  private final Mechanism2d m_mech2d = new Mechanism2d(100, 100);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private MechanismLigament2d elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", 1, 45, 10, new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d stingerMech2d =
      elevatorMech2d.append(
          new MechanismLigament2d(
              "Stinger", Units.metersToInches(1), -45, 10, new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d wristMech2d =
      stingerMech2d.append(new MechanismLigament2d("Wrist", 1, 0, 10, new Color8Bit(Color.kRed)));

  private double elevatorMinLengthInches = 10;
  private double stingerMinLengthInches = 15;

  private double elevatorLength = elevatorMinLengthInches;
  private double stingerlength = stingerMinLengthInches;

  private SimulatedMechanism() {}

  public static SimulatedMechanism getInstance() {
    if (instance == null) {
      return new SimulatedMechanism();
    } else {
      return instance;
    }
  }

  public void setElevatorLengthInches(double lengthInches) {
    Logger.getInstance()
        .recordOutput("elevator/lengthMeters set", elevatorMinLengthInches + lengthInches);
    elevatorLength = elevatorMinLengthInches + lengthInches;
  }

  public void setStingerLengthInches(double lengthInches) {
    Logger.getInstance()
        .recordOutput("elevator/stinger set", elevatorMinLengthInches + lengthInches);
    stingerlength = stingerMinLengthInches + lengthInches;
  }

  @Override
  public void periodic() {

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages

    // Update elevator visualization with simulated position
    elevatorMech2d.setLength(elevatorLength);

    // var stingerLength = stingerSim.getPositionMeters();
    // var stingerInches = Units.metersToInches(stingerLength);

    // stingerMech2d.setLength(stingerInches);

    // double wristAngleRad = wristSim.getAngleRads();
    // double wristAngleDegrees = Units.radiansToDegrees(wristAngleRad);

    // double speed = wristOutput;

    // wristMech2d.setAngle(wristAngleDegrees);

    Logger.getInstance().recordOutput("SimMech", m_mech2d);
  }
}
