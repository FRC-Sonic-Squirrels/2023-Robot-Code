// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.stinger.Stinger;

public class StingerManualControl extends CommandBase {

  private Stinger stinger;
  private DoubleSupplier axisController;

  /** Creates a new StingerManualControl. */
  public StingerManualControl(Stinger stinger, DoubleSupplier axisController) {

    this.stinger = stinger;
    this.axisController = axisController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double joystickValue = axisController.getAsDouble();

    // dead zone
    if (Math.abs(joystickValue) > 0.1) {
      stinger.setPercentOutput(joystickValue);
    } else {
      stinger.setPercentOutput(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
