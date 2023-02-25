// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.stinger.Stinger;

public class StingerSetExtension extends CommandBase {

  private Stinger stinger;
  private double extensionInches;

  // debounce: a statement must be true for a specific amount of time before it is viewed as true by
  // the code
  private double debounce;

  private double velocity;
  private double time;
  private boolean changeConstraints;

  private Trigger atHeight;

  /** Creates a new StingerSetHeight. */
  public StingerSetExtension(Stinger stinger, double extensionInches) {

    // "this" calls the other constructor with six different parameters
    this(stinger, extensionInches, false, 0.0, 0.0, 0.1);
  }

  public StingerSetExtension(Stinger stinger, double extensionInches, double debounce) {

    this(stinger, extensionInches, false, 0.0, 0.0, debounce);
  }

  public StingerSetExtension(
      Stinger stinger, double extensionInches, double velocity, double time) {

    this(stinger, extensionInches, true, velocity, time, 0.1);
  }

  public StingerSetExtension(
      Stinger stinger, double extensionInches, double velocity, double time, double debounce) {

    this(stinger, extensionInches, true, velocity, time, debounce);
  }

  // Called by all other constructors, puts in default values from previous constructor if not
  // specified
  public StingerSetExtension(
      Stinger stinger,
      double extensionInches,
      boolean changeConstraints,
      double velocity,
      double time,
      double debounce) {

    this.stinger = stinger;
    this.extensionInches = extensionInches;
    this.changeConstraints = changeConstraints;
    this.velocity = velocity;
    this.time = time;
    this.debounce = debounce;

    addRequirements(stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // checks to see if stinger has met target extension for more than 0.1 seconds
    // atHeight = new Trigger(() -> stinger.isAtExtension(extensionInches)).debounce(debounce);

    // if (changeConstraints) {
    //   stinger.setMotionProfileConstraintsTime(velocity, time);
    // }

    // extends stinger
    stinger.setExtensionInches(extensionInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // dont stop because that ends positional control
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO add this back in
    // make sure that the default command kicking in doesnt cause issues
    // return stinger.isAtExtension(extensionInches);
    return Math.abs(stinger.getExtensionInches() - extensionInches) < 0.3;
  }
}

