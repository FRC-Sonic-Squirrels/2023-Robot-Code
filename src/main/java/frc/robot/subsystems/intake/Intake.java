// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public final double MAX_VOLTAGE = 11.0;

  private final double INTAKE_CONE_INVERT = 1.0;
  private final double INTAKE_CUBE_INVERT = -1.0;

  private final double OUTTAKE_CONE_INVERT = -INTAKE_CONE_INVERT;
  private final double OUTTAKE_CUBE_INVERT = -INTAKE_CUBE_INVERT;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);

  private TunableNumber velocityThreshold = new TunableNumber("intake/velocityThreshold", 200);
  private TunableNumber currentThreshold = new TunableNumber("intake/currentThreshold", 100);

  private TunableNumber velocityThresholdCone =
      new TunableNumber("intake/CONEvelocityThreshold", 200);
  private TunableNumber currentThresholdCone =
      new TunableNumber("intake/CONEcurrentThreshold", 100);

  private double filteredVelocity = 0.0;
  private double filteredStatorCurrent = 0.0;

  /** Creates a new Intake */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.intakeVelocityRPM));
    filteredStatorCurrent = currentFilter.calculate(inputs.intakeStatorCurrent);

    Logger.getInstance().recordOutput("Intake/filteredStatorCurrent", filteredStatorCurrent);

    Logger.getInstance().recordOutput("Intake/filteredVelocity", filteredVelocity);

    Logger.getInstance().recordOutput("Intake/isStalled", isStalled());

    Logger.getInstance().recordOutput("Intake/CONEIsStalled", isStalledForCone());
  }

  /** Run the intake intake at the specified percentage. */
  public void runIntakePercent(double percent) {
    io.setIntakeVoltage(percent * MAX_VOLTAGE);
  }

  public void stop() {
    runIntakePercent(0.0);
  }

  public void intakeCone(double percent) {
    runIntakePercent(percent * INTAKE_CONE_INVERT);
  }

  public void intakeCube(double percent) {
    runIntakePercent(percent * INTAKE_CUBE_INVERT);
  }

  public void outTakeCone(double percent) {
    runIntakePercent(percent * OUTTAKE_CONE_INVERT);
  }

  public void outTakeCube(double percent) {
    runIntakePercent(percent * OUTTAKE_CUBE_INVERT);
  }

  // this needs to be called repeatedly because it uses a filter.
  // I.E calling this once will not return an accurate result
  // need to call this in an isFinished or a .until()
  public boolean isStalled() {
    // var filteredCurrent = StallDetectionFilter.calculate(Math.abs(inputs.intakeStatorCurrent));
    // var velocity = inputs.intakeVelocityRPM;

    // double minStallCurrent = 1;
    // double maxStallVelocity = 50;

    // return (filteredCurrent >= minStallCurrent) && (velocity <= maxStallVelocity);

    // FIXME: need to check these numbers

    return (filteredVelocity <= velocityThreshold.get()
        && (filteredStatorCurrent >= currentThreshold.get() || filteredStatorCurrent <= -2));
  }

  public boolean isStalledForCone() {

    return (filteredVelocity <= velocityThresholdCone.get()
        && (filteredStatorCurrent >= currentThresholdCone.get() || filteredStatorCurrent <= -2));
  }

  public void outtakeConeWithRPM(double speed) {}

  public void intakeConeWithRPM(double speed) {}

  public double getSpeedRPM() {
    return inputs.intakeVelocityRPM;
  }
}
