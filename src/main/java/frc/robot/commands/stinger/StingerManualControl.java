// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;
import java.util.function.DoubleSupplier;

public class StingerManualControl extends CommandBase {

  private Stinger stinger;
  private DoubleSupplier axisController;
  private Elevator elevator;

  /** Creates a new StingerManualControl. */
  public StingerManualControl(Stinger stinger, Elevator elevator, DoubleSupplier axisController) {

    this.stinger = stinger;
    this.axisController = axisController;

    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double joystickValue = axisController.getAsDouble() * 0.2;

    if (elevator.getHeightInches() < 3) {
      stinger.setPercentOutput(0.0);
      return;
    }

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
