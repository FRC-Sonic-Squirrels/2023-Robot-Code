// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.GamePiece;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.limelight.LimelightIO.LimelightIOInputs;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private final LimelightIO io;
  private final LimelightIOInputs inputs = new LimelightIOInputs();

  private Drivetrain drive;

  private GamePiece detectedGamePiece;
  private double targetYawDegrees;
  private double targetPitchDegrees;
  private double cubeDistanceMeters;
  private Pose2d cubePoseMeters;

  /** Creates a new Limelight. */
  public Limelight(LimelightIO io, Drivetrain drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Limelight", inputs);

    detectedGamePiece = inputs.classID != 0 ? GamePiece.CUBE : GamePiece.CONE;

    if (inputs.validTarget) {

      targetYawDegrees =
          -inputs.xOffset
              // / Constants.Limelight.RESOLUTION.getX() * Constants.Limelight.FOV.getX()
              + Math.toDegrees(Constants.Limelight.LIMELIGHT_POSE.getRotation().getZ());
      targetPitchDegrees =
          inputs.yOffset
              +
              // / Constants.Limelight.RESOLUTION.getY() * Constants.Limelight.FOV.getY()
              (90.0 + Math.toDegrees(Constants.Limelight.LIMELIGHT_POSE.getRotation().getY()));

      cubeDistanceMeters =
          (Constants.Limelight.LIMELIGHT_POSE.getZ()
                  - Constants.GAME_PIECE_DIMENSIONS.CUBE_LENGTH_METERS / 2)
              * Math.tan(Math.toRadians(targetPitchDegrees));

      // cubePoseMeters =
      //     drive
      //         .getPose()
      //         .transformBy(
      //             new Transform2d(
      //                 new Translation2d(
      //                     cubeDistanceMeters * Math.cos(Math.toRadians(targetYawDegrees)),
      //                     cubeDistanceMeters * Math.sin(Math.toRadians(targetYawDegrees))),
      //                 new Rotation2d(0)));

    } else {
      targetYawDegrees =
          new Rotation2d(
                  cubePoseMeters.getX() - drive.getPose().getX(),
                  cubePoseMeters.getY() - drive.getPose().getY())
              .getDegrees();
      cubeDistanceMeters =
          Math.sqrt(
              Math.pow(drive.getPose().getX() - cubePoseMeters.getX(), 2)
                  + Math.pow(drive.getPose().getY() - cubePoseMeters.getY(), 2));
    }

    Logger.getInstance()
        .recordOutput(
            "Limelight/totalLatencyMs", inputs.pipelineLatencyMs + inputs.captureLatencyMs);
    Logger.getInstance().recordOutput("Limelight/detectedGamePiece", detectedGamePiece.toString());
    Logger.getInstance().recordOutput("Limelight/targetAngleDegrees", targetYawDegrees);
    Logger.getInstance().recordOutput("Limelight/targetPitchDegrees", targetPitchDegrees);
    Logger.getInstance().recordOutput("Limelight/cubeDistanceMeters", cubeDistanceMeters);
    Logger.getInstance().recordOutput("Limelight/cubePoseMeters", cubePoseMeters);
    Logger.getInstance()
        .recordOutput(
            "Limelight/cubeOffset",
            new Pose2d(
                new Translation2d(
                    cubeDistanceMeters * Math.cos(Math.toRadians(targetYawDegrees)),
                    cubeDistanceMeters * Math.sin(Math.toRadians(targetYawDegrees))),
                new Rotation2d(0)));
  }

  /**
   * Changes the mode of the leds
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * 0: use the LED Mode set in the current pipeline
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * 1: force off
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * 2: force blink
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * 3: force on
   */
  public void ledMode(double mode) {
    io.ledMode(mode);
  }

  public boolean isValidTarget() {
    return inputs.validTarget;
  }

  public GamePiece getDetectedGamepiece() {
    return detectedGamePiece;
  }

  public Rotation2d getTargetYaw() {
    return new Rotation2d(Math.toRadians(targetYawDegrees));
    // return new Rotation2d(Math.toRadians(targetYawDegrees));
  }

  public Rotation2d getTargetPitch() {
    return new Rotation2d(Math.toRadians(targetPitchDegrees));
  }

  public double getDistanceToGroundCube() {
    return cubeDistanceMeters;
  }

  public Pose2d getCubePoseMeters() {
    return cubePoseMeters;
    // return cubePoseMeters;
  }
}
