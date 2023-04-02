// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.team2930.driverassist.GridPositionHandler.PoseAndHeading;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GenerateContinuouslyAndFollowPath extends CommandBase {
  private final Timer timer = new Timer();
  private PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final PPHolonomicDriveController controller;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final boolean useKinematics;
  private final Field2d field = new Field2d();

  private final Drivetrain drivetrain;

  private final PathConstraints pathConstraints;

  private final PoseAndHeading targetPoint;

  private final Timer regenerationTimer = new Timer();

  private final Timer initialWaitTimer = new Timer();

  private static final TunableNumber timeBetweenRegeneration =
      new TunableNumber("pathFollow/timeBeteweenRegeneration", 0.1);

  private static final TunableNumber initialTimeToWait =
      new TunableNumber("pathFollow/initialTimeToWait", 0.75);

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
   * @param requirements The subsystems to require.
   */
  private GenerateContinuouslyAndFollowPath(
      Drivetrain drivetrain,
      PathConstraints pathConstraints,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      PoseAndHeading targetPose,
      Subsystem... requirements) {
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.useKinematics = true;

    this.drivetrain = drivetrain;

    this.pathConstraints = pathConstraints;

    this.trajectory = null;

    this.targetPoint = targetPose;

    addRequirements(requirements);
  }

  /**
   * @param drivetrain
   * @param poses
   * @param pathConstraints
   * @param firstPathPose used to optimize the initial heading of the robot, passing null is okay
   *     and will result in a default heading of 180 (towards alliance wall)
   */
  public GenerateContinuouslyAndFollowPath(
      Drivetrain drivetrain, PoseAndHeading targetPose, PathConstraints pathConstraints) {
    this(
        drivetrain,
        pathConstraints,
        drivetrain::getPose,
        DrivetrainConstants.KINEMATICS,
        drivetrain.getAutoXController(),
        drivetrain.getAutoYController(),
        drivetrain.getAutoThetaController(),
        drivetrain::setSwerveModuleStates,
        targetPose,
        drivetrain);
  }

  @Override
  public void initialize() {

    List<PathPoint> pathPoints = new ArrayList<PathPoint>();

    var currentSpeeds = drivetrain.getDriverFieldRelativeInput();

    double linearVel =
        Math.sqrt(
            (currentSpeeds.getX() * currentSpeeds.getX())
                + (currentSpeeds.getY() * currentSpeeds.getY()));

    Pose2d currentPose = drivetrain.getPose();

    double heading = Math.toRadians(180);

    Logger.getInstance().recordOutput("DriverAssist/linearVel", linearVel);

    if (linearVel > 1) {
      var distanceVector = new Translation2d(currentSpeeds.getX(), currentSpeeds.getY());
      heading = distanceVector.getAngle().getRadians();

    } else {
      var distanceX = targetPoint.pose.getX() - currentPose.getX();
      var distanceY = targetPoint.pose.getY() - currentPose.getY();

      var distanceVector = new Translation2d(distanceX, distanceY);

      heading = distanceVector.getAngle().getRadians();
    }

    pathPoints.add(
        new PathPoint(
            currentPose.getTranslation(),
            Rotation2d.fromRadians(heading),
            currentPose.getRotation(),
            linearVel));

    pathPoints.add(poseAndHeadingToPathPoint(targetPoint));

    this.trajectory = PathPlanner.generatePath(this.pathConstraints, pathPoints);

    Trajectory displayTrajectory = decimateTrajectory(trajectory, 30);

    SmartDashboard.putData("PPSwerveControllerCommand_field", this.field);
    this.field.getObject("traj").setTrajectory(displayTrajectory);

    Logger.getInstance()
        .recordOutput("PathFollowing/continuouslyRegeneratingPath", displayTrajectory);

    PathPlannerServer.sendActivePath(this.trajectory.getStates());

    this.timer.reset();
    this.timer.start();

    this.regenerationTimer.reset();
    regenerationTimer.start();

    initialWaitTimer.reset();
    initialWaitTimer.start();
  }

  @Override
  public void execute() {
    if (initialWaitTimer.get() > initialTimeToWait.get()) {

      if (regenerationTimer.get() >= timeBetweenRegeneration.get()) {
        List<PathPoint> regeneratedPoints = new ArrayList<PathPoint>();
        regeneratedPoints.add(getCurrentDrivetrainStateAsPathPoint());
        regeneratedPoints.add(poseAndHeadingToPathPoint(targetPoint));

        this.trajectory = PathPlanner.generatePath(this.pathConstraints, regeneratedPoints);

        var displayTrajectory = decimateTrajectory(trajectory, 20);

        Logger.getInstance()
            .recordOutput("PathFollowing/continuouslyRegeneratingPath", displayTrajectory);

        regenerationTimer.reset();
        timer.reset();
      }
    }

    Logger.getInstance()
        .recordOutput("PathFollowing/pathTotalTime", trajectory.getTotalTimeSeconds());

    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

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

    // if (interrupted) {
    //   if (useKinematics) {
    //     this.outputModuleStates.accept(
    //         this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
    //   } else {
    //     this.outputChassisSpeeds.accept(new ChassisSpeeds());
    //   }
    // }

    // drivetrain.stop();
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    // TODO: could experiment with making this longer and having the end condition as the robot pose
    // being
    // close to the end pose or total time plus some abritary amount
    // this would help with super short trajectries which have to rotate a large amount and as such
    // cannot fully rotate before the command ends
    return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
  }

  private static PathPoint poseAndHeadingToPathPoint(PoseAndHeading poseAndHeading) {
    return new PathPoint(
        poseAndHeading.pose.getTranslation(),
        poseAndHeading.heading,
        poseAndHeading.pose.getRotation());
  }

  private PathPoint getCurrentDrivetrainStateAsPathPoint() {
    var currentPose = drivetrain.getPose();

    var currentSpeeds = drivetrain.getCurrentForwardKinematicsChassisSpeeds();

    double linearVel =
        Math.sqrt(
            (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
                + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));

    double heading = Math.toRadians(180);

    Logger.getInstance().recordOutput("DriverAssist/linearVel", linearVel);

    // if (linearVel > 3.5) {
    //   var distanceVector =
    //       new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    //   heading = distanceVector.getAngle().getRadians();

    // } else {
    var distanceX = targetPoint.pose.getX() - currentPose.getX();
    var distanceY = targetPoint.pose.getY() - currentPose.getY();

    var distanceVector = new Translation2d(distanceX, distanceY);

    heading = distanceVector.getAngle().getRadians();
    // }

    return new PathPoint(
        currentPose.getTranslation(),
        Rotation2d.fromRadians(heading),
        currentPose.getRotation(),
        linearVel);
  }

  public static Trajectory decimateTrajectory(Trajectory detailed, int modulus) {

    if (detailed == null) {
      return detailed;
    }

    List<State> states = new ArrayList<State>();

    for (int i = 0; i < detailed.getStates().size(); ++i) {
      var s = detailed.getStates().get(i);

      if (i == 0 || i == detailed.getStates().size() || (i % modulus == 0)) {
        states.add(
            new State(
                s.timeSeconds,
                s.velocityMetersPerSecond,
                s.accelerationMetersPerSecondSq,
                s.poseMeters,
                s.curvatureRadPerMeter));
      }
    }

    return new Trajectory(states);
  }
}
