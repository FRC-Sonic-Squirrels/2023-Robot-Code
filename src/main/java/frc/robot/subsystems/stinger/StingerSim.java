package frc.robot.subsystems.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SimMechanism.SimulatedMechanism;
import org.littletonrobotics.junction.Logger;

public class StingerSim implements StingerIO {

  SimulatedMechanism mechanism;
  // maybe multiply by 2 here to simulate the cascade effect of moving 2 inches for every inch u
  // spin
  private static final double gearing = 10.0;
  private static final double drumRadius = Units.inchesToMeters(2.0);
  private static final double carriageMass = 4.0; // kg

  private static final double minExtension = Units.inchesToMeters(0.0);

  private static final double maxExtension = Units.inchesToMeters(30); // 47


  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * drumRadius / 4096;

  private final DCMotor gearboxMotors = DCMotor.getFalcon500(1);

  edu.wpi.first.wpilibj.simulation.ElevatorSim simStinger =
      new edu.wpi.first.wpilibj.simulation.ElevatorSim(
          gearboxMotors, gearing, carriageMass, drumRadius, minExtension, maxExtension, false);

  ProfiledPIDController controller =
      new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(40, 80));


  private boolean closedLoop = false;
  private double targetHeightInches = 0.0;
  private double desiredVoltsOpenLoop = 0.0;
  private double currentExtensionInches = 0.0;

  public StingerSim() {
    mechanism = SimulatedMechanism.getInstance();
  }

  @Override
  public void updateInputs(StingerIOInputs inputs) {
    double controlEffort = 0.0;
    if (closedLoop) {
      controlEffort = controller.calculate(inputs.StingerExtensionInches, targetHeightInches);

      simStinger.setInput(controlEffort);
    } else {
      controlEffort = desiredVoltsOpenLoop;
      simStinger.setInput(controlEffort);
    }

    simStinger.update(0.020);

    mechanism.setStingerLengthInches(Units.metersToInches(simStinger.getPositionMeters()));

    Logger.getInstance().recordOutput("stinger/controlEffort", controlEffort);
    Logger.getInstance().recordOutput("stinger/actual controller kp", controller.getP());

    inputs.StingerExtensionInches = Units.metersToInches(simStinger.getPositionMeters());
    currentExtensionInches = Units.metersToInches(simStinger.getPositionMeters());
    inputs.StingerTargetExtensionInches = targetHeightInches;
  }

  public void setPercent(double percent) {
    percent = MathUtil.clamp(percent, -1.0, 1.0);
    setStingerVoltage(percent * 12);
  }

  @Override
  public void setStingerVoltage(double volts) {
    closedLoop = false;
    var num = MathUtil.clamp(volts, -12, 12);
    desiredVoltsOpenLoop = num;
  }

  @Override
  public void setExtensionInches(double targetHeightInches) {
    closedLoop = true;
    // controller.reset(currentExtensionInches);
    this.targetHeightInches = targetHeightInches;
  }

  @Override
  public void setPIDConstraints(double kF, double kP, double kI, double kD) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  @Override
  public void setMotionProfileConstraints(
      double cruiseVelocityInchesPerSecond, double accelerationInchesPerSecondSquared) {
    controller.setConstraints(
        new Constraints(cruiseVelocityInchesPerSecond, accelerationInchesPerSecondSquared));
  }
}
