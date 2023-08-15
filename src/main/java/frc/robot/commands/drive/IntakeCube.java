// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;
import java.util.function.DoubleSupplier;

public class IntakeCube extends CommandBase {
  /** Creates a new IntakeCube. */
  private final Drivetrain drive;

  private final Elevator elevator;
  private final Stinger stinger;
  private final Intake intake;
  private DoubleSupplier xOffset;
  private DoubleSupplier yOffset;

  private Double distToCube;
  private Double cubeAngle;

  public IntakeCube(
      DoubleSupplier xOffset,
      DoubleSupplier yOffset,
      Drivetrain drive,
      Elevator elevator,
      Stinger stinger,
      Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.elevator = elevator;
    this.stinger = stinger;
    this.intake = intake;
    this.xOffset = xOffset;
    this.yOffset = yOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(0, 0, 0);
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
