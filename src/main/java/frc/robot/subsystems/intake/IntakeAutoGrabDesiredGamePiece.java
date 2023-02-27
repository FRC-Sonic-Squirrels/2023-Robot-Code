// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;

public class IntakeAutoGrabDesiredGamePiece extends ConditionalCommand {
  public IntakeAutoGrabDesiredGamePiece(Intake intake) {
    super(
        new IntakeGrabCone(intake),
        new IntakeGrabCube(intake),
        () -> RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE);
  }
}
