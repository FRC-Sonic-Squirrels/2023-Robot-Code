// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.stinger.Stinger;

public class StingerSetHeight extends CommandBase {

  private Stinger stinger;
  private double heightInches;

  // debounce: a statement must be true for a specific amount of time before it is viewed as true by the code
  private double debounce;

  private double velocity;
  private double time;
  private boolean changeConstraints;

  /** Creates a new StingerSetHeight. */
  public StingerSetHeight(Stinger stinger, double heightInches) {

    this.stinger = stinger;
    this.heightInches = heightInches;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stinger);
  }

  public StingerSetHeight(Stinger stinger, double heightInches, double debounce) {

    this.stinger = stinger;
    this.heightInches = heightInches;
    this.debounce = debounce;

    addRequirements(stinger);
  }

  public StingerSetHeight(Stinger stinger, double heightInches, double velocity, double time) {

    this.stinger = stinger;
    this.heightInches = heightInches;

    this.velocity = velocity;
    this.time = time;
    changeConstraints = true;

    addRequirements(stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
