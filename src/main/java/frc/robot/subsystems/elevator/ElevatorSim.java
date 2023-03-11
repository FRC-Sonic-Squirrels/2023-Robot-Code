// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SimMechanism.SimulatedMechanism;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {
  SimulatedMechanism mechanism;

  private static final double kElevatorKp = 5.0;

  private static final double kElevatorGearing = 10.0;
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 4.0; // kg

  private static final double kMinElevatorHeight = Units.inchesToMeters(0.0);
  private static final double kMaxElevatorHeight = Units.inchesToMeters(Constants.Elevator.MAX_HEIGHT_INCHES + 1); // 24*2

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;

  private final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);

  edu.wpi.first.wpilibj.simulation.ElevatorSim simElevator =
      new edu.wpi.first.wpilibj.simulation.ElevatorSim(
          elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeight,
          kMaxElevatorHeight,
          false);

  ProfiledPIDController controller =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(40, 80));

  private boolean closedLoop = false;
  private double targetHeightInches = 0.0;
  private double desiredVoltsOpenLoop = 0.0;

  private double currentHeightInches = 0.0;

  public ElevatorSim() {
    mechanism = SimulatedMechanism.getInstance();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double controlEffort = 0.0;
    if (closedLoop) {
      controlEffort = controller.calculate(inputs.ElevatorHeightInches, targetHeightInches);

      simElevator.setInput(controlEffort);
    } else {
      controlEffort = desiredVoltsOpenLoop;
      simElevator.setInput(controlEffort);
    }

    simElevator.update(0.020);

    mechanism.setElevatorLengthInches(Units.metersToInches(simElevator.getPositionMeters()));

    // this would have to communicate among all sim systems
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(simElevator.getCurrentDrawAmps()));

    Logger.getInstance().recordOutput("elevator/controlEffort", controlEffort);
    Logger.getInstance().recordOutput("elevator/actual controller kp", controller.getP());

    inputs.ElevatorHeightInches = Units.metersToInches(simElevator.getPositionMeters());
    currentHeightInches = Units.metersToInches(simElevator.getPositionMeters());
    inputs.ElevatorTargetHeightInches = targetHeightInches;
  }

  public void setPercent(double percent) {
    percent = MathUtil.clamp(percent, -1.0, 1.0);
    setElevatorVoltage(percent * 12);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    closedLoop = false;
    var num = MathUtil.clamp(volts, -12, 12);
    desiredVoltsOpenLoop = num;
  }

  @Override
  public void setHeightInches(double targetHeightInches) {
    closedLoop = true;
    // controller.reset(currentHeightInches);
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
