// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  private final double MAX_VOLTAGE = 12.0;

  private final boolean EXTENDED_BOOL = true;
  private final boolean RETRACTED_BOOL = false;

  // TODO figure out which way the motor would spin for every action
  private final double INTAKE_CONE_INVERT = -1;
  private final double INTAKE_CUBE_INVERT = -1;

  private final double OUTTAKE_CONE_INVERT = 1;
  private final double OUTTAKE_CUBE_INVERT = 1;

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
    io.setIntakeVoltage(percent * MAX_VOLTAGE);
  }

  public void stop() {
    runIntakePercent(0.0);
  }

  public void extend() {
    io.setExtended(EXTENDED_BOOL);
  }

  public void retract() {
    io.setExtended(RETRACTED_BOOL);
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

  public Command extendCommand() {
    return new InstantCommand(
        () -> {
          extend();
        });
  }

  public Command retractCommand() {
    return new InstantCommand(
        () -> {
          retract();
        });
  }
}
