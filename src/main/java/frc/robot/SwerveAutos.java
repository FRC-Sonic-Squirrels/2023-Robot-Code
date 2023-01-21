package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPath;
import frc.robot.commands.FollowPathWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;

import java.util.HashMap;
import java.util.List;

public class SwerveAutos {
  private Drivetrain drivetrain;
  private Intake intake;

  public SwerveAutos(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  public Command testPath2mForward() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward",
            AUTO_TEST_MAX_SPEED_METERS_PER_SECOND,
            AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath2mForward180() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward180",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath3mForward360() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "3mForward360",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command curve() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "curve",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command rotateAroundPoint() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "rotateAroundPoint",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath2mForwardWithIntake() {

    HashMap<String, Command> EVENT_MAP = new HashMap<>();
    
    EVENT_MAP.put("event1", Commands.runOnce(intake::extend, intake)
    .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.5), intake)));
    EVENT_MAP.put("event2", Commands.runOnce(intake::retract, intake)
    .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.0), intake)));

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "testPath2mForwardWithIntake",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), EVENT_MAP));
  }

  public Command testPathSquareGroup() {
    List<PathPlannerTrajectory> pathGroup1 =
        PathPlanner.loadPathGroup(
            "squarePathGroup",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    return new SequentialCommandGroup(
        new FollowPath(pathGroup1.get(0), drivetrain, true),
        new FollowPath(pathGroup1.get(1), drivetrain, true),
        new FollowPath(pathGroup1.get(2), drivetrain, true),
        new FollowPath(pathGroup1.get(3), drivetrain, true));
  }

  public Command forwardLeft() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "forwardLeft",
            AUTO_MAX_SPEED_METERS_PER_SECOND*0.1,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }
}
