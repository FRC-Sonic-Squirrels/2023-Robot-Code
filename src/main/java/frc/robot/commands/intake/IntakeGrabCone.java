// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeGrabCone extends CommandBase {
  /** Creates a new IntakeGrabCone. */
  Intake intake;

  double speed;

  public IntakeGrabCone(Intake intake) {
    this(intake, 0.8);
  }

  public IntakeGrabCone(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if we use the pivot then we will have to extend and retrace as well
    // intake.extend();
    intake.intakeConeWithRPM(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if we use the pivot then we will have to extend and retrace as well
    // intake.retract();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
