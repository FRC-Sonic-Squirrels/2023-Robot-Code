// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GenerateAndFollowPath extends CommandBase {
  private final Timer timer = new Timer();
  private PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final PPHolonomicDriveController controller;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final boolean useKinematics;
  private final boolean useAllianceColor;
  private final Field2d field = new Field2d();

  private final Drivetrain drivetrain;

  private final List<PathPoint> pathWaypoints;
  private final PathConstraints pathConstraints;

  private Pose2d firstPathPose;

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  private GenerateAndFollowPath(
      Drivetrain drivetrain,
      List<PathPoint> pathWaypoints,
      PathConstraints pathConstraints,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.useKinematics = true;
    this.useAllianceColor = useAllianceColor;

    this.drivetrain = drivetrain;
    this.pathWaypoints = pathWaypoints;
    this.pathConstraints = pathConstraints;

    this.trajectory = null;

    addRequirements(requirements);
  }

  public GenerateAndFollowPath(
      Drivetrain drivetrain,
      List<PathPoint> poses,
      PathConstraints pathConstraints,
      boolean useAllianceColor) {
    this(
        drivetrain,
        poses,
        pathConstraints,
        drivetrain::getPose,
        DrivetrainConstants.KINEMATICS,
        drivetrain.getAutoXController(),
        drivetrain.getAutoYController(),
        drivetrain.getAutoThetaController(),
        drivetrain::setSwerveModuleStates,
        useAllianceColor,
        drivetrain);
  }

  public GenerateAndFollowPath(
      Drivetrain drivetrain,
      List<PathPoint> poses,
      PathConstraints pathConstraints,
      Pose2d firstPathPose,
      boolean useAllianceColor) {
    this(
        drivetrain,
        poses,
        pathConstraints,
        drivetrain::getPose,
        DrivetrainConstants.KINEMATICS,
        drivetrain.getAutoXController(),
        drivetrain.getAutoYController(),
        drivetrain.getAutoThetaController(),
        drivetrain::setSwerveModuleStates,
        useAllianceColor,
        drivetrain);

    this.firstPathPose = firstPathPose;
  }

  @Override
  public void initialize() {

    List<PathPoint> pathPoints = new ArrayList<PathPoint>();

    // ChassisSpeeds currentSpeeds = drivetrain.getCurrentChassisSpeeds();

    // double linearVel =
    //     Math.sqrt(
    //         (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
    //             + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));

    // Pose2d currentPose = drivetrain.getPose();
    // Pose2d initialPose =
    //     new Pose2d(currentPose.getX() - 0.5, currentPose.getY(), currentPose.getRotation());

    // PathPoint initialPoint =
    //     new PathPoint(
    //         initialPose.getTranslation(),
    //         Rotation2d.fromDegrees(180),
    //         initialPose.getRotation(),
    //         linearVel);

    // pathPoints.add(initialPoint);

    // pathPoints.add(
    //     PathPoint.fromCurrentHolonomicState(
    //         drivetrain.getPose(), drivetrain.getCurrentChassisSpeeds()));

    ChassisSpeeds currentSpeeds = drivetrain.getCurrentChassisSpeeds();

    double linearVel =
        Math.sqrt(
            (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
                + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));

    // pathPoints.add(
    //     new PathPoint(
    //         drivetrain.getPose(),
    //         Rotation2d.fromDegrees(180),
    //         drivetrain.getPose().getRotation(),
    //         linearVel));

    this.timer.reset();
    this.timer.start();

    Pose2d currentPose = drivetrain.getPose();

    // FIXME: this is okay for if the robot is still but if its in motion use the chassis speeds for
    // heading
    // NOTE: I think u have to swap the X and Y in the atan2 arguments because of how the field is
    // oriented
    double heading;
    if (this.firstPathPose != null) {
      heading =
          Math.atan2(
              firstPathPose.getY() - currentPose.getY(),
              firstPathPose.getX() - firstPathPose.getX());
    } else {
      heading = Math.toRadians(180);
    }

    pathPoints.add(
        new PathPoint(
            currentPose.getTranslation(),
            Rotation2d.fromRadians(heading),
            currentPose.getRotation(),
            linearVel));

    Logger.getInstance()
        .recordOutput("DriverAssist/GridPosition/initialHeading", Math.toDegrees(heading));

    // FIXME: heading is hard coded
    // pathPoints.add(
    //     new PathPoint(
    //         drivetrain.getPose().getTranslation(),
    //         Rotation2d.fromDegrees(180),
    //         drivetrain.getPose().getRotation(),
    //         linearVel));

    pathPoints.addAll(this.pathWaypoints);

    this.trajectory = PathPlanner.generatePath(this.pathConstraints, pathPoints);

    Logger.getInstance()
        .recordOutput(
            "Odometry/initial heading?", trajectory.getInitialState().curvatureRadPerMeter);

    SmartDashboard.putData("PPSwerveControllerCommand_field", this.field);
    this.field.getObject("traj").setTrajectory(this.trajectory);

    PathPlannerServer.sendActivePath(this.trajectory.getStates());

    Logger.getInstance().recordOutput("Odometry/generatedPath", this.trajectory);
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

    // if (useAllianceColor && trajectory.fromGUI) {
    //   desiredState =
    //       PathPlannerTrajectory.transformStateForAlliance(
    //           desiredState, DriverStation.getAlliance());
    // }
    // if (useAllianceColor) {
    //   desiredState =
    //       PathPlannerTrajectory.transformStateForAlliance(
    //           desiredState, DriverStation.getAlliance());
    // }

    Pose2d currentPose = this.poseSupplier.get();
    this.field.setRobotPose(currentPose);

    PathPlannerServer.sendPathFollowingData(
        new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        currentPose);

    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_rotationError",
        currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

    Logger.getInstance()
        .recordOutput("Odometry/vxMetersPerSecond", targetChassisSpeeds.vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Odometry/vyMetersPerSecond", targetChassisSpeeds.vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput(
            "Odometry/omegaradPerSecond",
            Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));

    if (this.useKinematics) {
      SwerveModuleState[] targetModuleStates =
          this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

      this.outputModuleStates.accept(targetModuleStates);
    } else {
      this.outputChassisSpeeds.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted) {
      if (useKinematics) {
        this.outputModuleStates.accept(
            this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
      } else {
        this.outputChassisSpeeds.accept(new ChassisSpeeds());
      }
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
  }
}
