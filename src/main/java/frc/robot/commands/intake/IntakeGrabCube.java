// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeGrabCube extends CommandBase {
  /** Creates a new IntakeGrabCube. */
  Intake intake;

  double speed;

  public IntakeGrabCube(Intake intake) {

    this(intake, 0.7);
  }

  public IntakeGrabCube(Intake intake, double speed) {

    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeCube(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: check for motor stall or limit switch
    return false;
  }
}
