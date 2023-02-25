// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;
import org.littletonrobotics.junction.Logger;

public class StingerFollowCurve extends CommandBase {

  private Stinger stinger;
  private Elevator elevator;
  private double stingerExtension;
  private double elevatorHeight;

  public StingerFollowCurve(Elevator elevator, Stinger stinger, double elevatorHeight) {
    this.elevator = elevator;
    this.stinger = stinger;
    this.elevatorHeight = elevatorHeight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stingerExtension =
        MathUtil.clamp(-(7 * (Math.sqrt(-elevator.getHeightInches() + 50) - 6)), 0, 30);
    Logger.getInstance().recordOutput("ActiveCommands/stingerFollowCurve", true);
    Logger.getInstance().recordOutput("StingerCurve/Extension", stingerExtension);
    Logger.getInstance().recordOutput("StingerCurve/Elevator", elevator.getHeightInches());
    // find elevator height through formula, set stinger extension to result
    stinger.setExtensionInches(stingerExtension);
    elevator.setHeightInches(elevatorHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/stingerFollowCurve", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: end with both stinger and elevator at setpoint
    return (elevator.isAtHeight(elevatorHeight));
  }
}
