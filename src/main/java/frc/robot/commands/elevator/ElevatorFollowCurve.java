// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;
import org.littletonrobotics.junction.Logger;

public class ElevatorFollowCurve extends CommandBase {

  private Stinger stinger;
  private Elevator elevator;
  private double elevatorHeight;

  public ElevatorFollowCurve(Elevator elevator, Stinger stinger) {
    this.elevator = elevator;
    this.stinger = stinger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorHeight =
        MathUtil.clamp(-Math.pow((1 / 7f * -stinger.getExtensionInches() + 6), 2) + 50, 0, 48);
    Logger.getInstance().recordOutput("ActiveCommands/elevatorFollowCurve", true);
    Logger.getInstance().recordOutput("ElevatorCurve/Height", elevatorHeight);
    Logger.getInstance().recordOutput("ElevatorCurve/Stinger", stinger.getExtensionInches());
    // find elevator height through formula, set stinger extension to result
    elevator.setHeightInches(elevatorHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/elevatorFollowCurve", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
