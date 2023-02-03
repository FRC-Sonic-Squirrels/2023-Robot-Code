// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.SimMechanism.SimulatedMechanism;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {
  SimulatedMechanism mechanism;

  ProfiledPIDController controller =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(20, 40));

  private boolean closedLoop = false;
  private double targetHeightInches = 0.0;

  public ElevatorSim() {
    mechanism = SimulatedMechanism.getInstance();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      double controlEffort =
          controller.calculate(mechanism.getElevatorPositionInches(), targetHeightInches);
      setElevatorVoltage(controlEffort);

      Logger.getInstance().recordOutput("elevator/controlEffort", controlEffort);
      Logger.getInstance().recordOutput("elevator/kp", controller.getP());
      // Logger.getInstance().recordOutput("elevator/kp", controller.);
    }

    inputs.ElevatorHeightInches = mechanism.getElevatorPositionInches();
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
    mechanism.setElevatorOutputVolts(num);
  }

  @Override
  public void setHeightInches(double targetHeightInches) {
    closedLoop = true;
    this.targetHeightInches = targetHeightInches;
  }

  @Override
  public void setPIDConstraints(double kF, double kP, double kI, double kD) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  @Override
  public void setMotionProfileConstraints(double cruiseVelocity, double desiredTimeToSpeed) {
    double acceleration = cruiseVelocity / desiredTimeToSpeed;

    controller.setConstraints(new Constraints(cruiseVelocity, acceleration));
  }
}
