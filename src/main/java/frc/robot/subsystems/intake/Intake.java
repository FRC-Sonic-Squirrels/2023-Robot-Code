// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  /** Creates a new Intake */
  public Intake(IntakeIO io) {
    this.io = io;
    retract();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }

  /** Run the intake intake at the specified percentage. */
  public void runIntakePercent(double percent) {
    io.setIntakeVoltage(percent * 12.0);
  }

  public void stop() {
    runIntakePercent(0.0);
  }

  public void extend() {
    io.setExtended(true);
  }

  public void retract() {
    io.setExtended(false);
  }
}
