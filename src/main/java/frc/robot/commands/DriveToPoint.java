// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.*;
import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends CommandBase {
  public static final String ROOT_TABLE = "Drive_To_Point";
  private final TunableNumber xControllerkP =
      new TunableNumber(ROOT_TABLE + "/xController/kP", 0.0);
  private final TunableNumber xControllerkI =
      new TunableNumber(ROOT_TABLE + "/xController/kI", 0.0);
  private final TunableNumber xControllerkD =
      new TunableNumber(ROOT_TABLE + "/xController/kD", 0.0);

  private final TunableNumber yControllerkP =
      new TunableNumber(ROOT_TABLE + "/yController/kP", 0.0);
  private final TunableNumber yControllerkI =
      new TunableNumber(ROOT_TABLE + "/yController/kI", 0.0);
  private final TunableNumber yControllerkD =
      new TunableNumber(ROOT_TABLE + "/yController/kD", 0.0);

  private final TunableNumber thetaControllerkP =
      new TunableNumber(ROOT_TABLE + "/thetaController/kP", 0.0);
  private final TunableNumber thetaControllerkI =
      new TunableNumber(ROOT_TABLE + "/thetaController/kI", 0.0);
  private final TunableNumber thetaControllerkD =
      new TunableNumber(ROOT_TABLE + "/thetaController/kD", 0.0);
  private final TunableNumber thetaControllerMaxVel =
      new TunableNumber(ROOT_TABLE + "/thetaController/maxVel", 0.0);
  private final TunableNumber thetaControllerMaxAccel =
      new TunableNumber(ROOT_TABLE + "/thetaController/maxAccel", 0.0);

  private final TunableNumber desiredVelocity =
      new TunableNumber(ROOT_TABLE + "/desiredVelocity", 0.0);

  Drivetrain drivetrain;
  // ProfiledPIDController xController;
  // ProfiledPIDController yController;
  // ProfiledPIDController thetaController;

  PIDController xController =
      new PIDController(xControllerkP.get(), xControllerkI.get(), xControllerkD.get());
  PIDController yController =
      new PIDController(yControllerkP.get(), yControllerkI.get(), yControllerkD.get());
  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaControllerkP.get(),
          thetaControllerkI.get(),
          thetaControllerkD.get(),
          new Constraints(thetaControllerMaxVel.get(), thetaControllerMaxAccel.get()));

  Pose2d targetPose;

  Pose2d poseTolerance;

  Translation2d poseError;
  Rotation2d rotationError;

  // public DriveToPoint(
  //     Drivetrain drivetrain,
  //     Pose2d targetPose,
  //     ProfiledPIDController xController,
  //     ProfiledPIDController yController,
  //     ProfiledPIDController thetaController,
  //     Pose2d poseTolerance) {

  //   this.xController = xController;
  //   this.yController = yController;
  //   this.thetaController = thetaController;

  //   this.poseTolerance = poseTolerance;
  //   this.targetPose = targetPose;
  // addRequirements(drivetrain)
  // Use addRequirements() here to declare subsystem dependencies.
  // }

  public DriveToPoint(Drivetrain drivetrain, Pose2d targetPose, Pose2d poseTolerance) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.poseTolerance = poseTolerance;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.enableFieldRelative();

    thetaController.reset(drivetrain.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.
    // if (m_firstRun) {
    //   m_thetaController.reset(currentPose.getRotation().getRadians());
    //   m_firstRun = false;
    // }

    // Calculate feedforward velocities (field-relative).
    double xFF = desiredVelocity.get() * targetPose.getRotation().getCos();
    double yFF = desiredVelocity.get() * targetPose.getRotation().getSin();
    double thetaFF =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    poseError = targetPose.relativeTo(currentPose).getTranslation();
    rotationError = targetPose.getRotation().minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());

    // Return next output.
    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double thetaVel = thetaFF;

    drivetrain.drive(xVel, yVel, thetaVel);

    // tunable number updating
    if (xControllerkP.hasChanged() || xControllerkI.hasChanged() || xControllerkD.hasChanged()) {
      xController.setPID(xControllerkP.get(), xControllerkI.get(), xControllerkD.get());
    }
    if (yControllerkP.hasChanged() || yControllerkI.hasChanged() || yControllerkD.hasChanged()) {
      yController.setPID(yControllerkP.get(), yControllerkI.get(), yControllerkD.get());
    }
    if (thetaControllerkP.hasChanged()
        || thetaControllerkI.hasChanged()
        || thetaControllerkD.hasChanged()
        || thetaControllerMaxVel.hasChanged()
        || thetaControllerMaxAccel.hasChanged()) {
      thetaController.setPID(
          thetaControllerkP.get(), thetaControllerkI.get(), thetaControllerkD.get());
      thetaController.setConstraints(
          new Constraints(thetaControllerMaxVel.get(), thetaControllerMaxAccel.get()));
    }

    // logging
    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual xController/kP", xController.getP());
    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual xController/kI", xController.getI());
    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual xController/kD", xController.getD());

    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual yController/kP", yController.getP());
    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual yController/kI", yController.getI());
    Logger.getInstance().recordOutput(ROOT_TABLE + "/actual yController/kD", yController.getD());

    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/actual thetaController/kP", thetaController.getP());
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/actual thetaController/kI", thetaController.getI());
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/actual thetaController/kD", thetaController.getD());

    // Logger doesnt support logging translation so make pose with dummy rotation value
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/pose error", new Pose2d(poseError, new Rotation2d()));
    Logger.getInstance()
        .recordOutput(ROOT_TABLE + "/rotation error degrees", rotationError.getDegrees());

    Logger.getInstance().recordOutput(ROOT_TABLE + "/pose tolerance", poseTolerance);

    Logger.getInstance().recordOutput(ROOT_TABLE + "/at reference", atReference());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atReference();
  }

  public boolean atReference() {
    final var eTranslate = poseError;
    final var eRotate = rotationError;
    final var tolTranslate = poseTolerance.getTranslation();
    final var tolRotate = poseTolerance.getRotation();

    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }
}
