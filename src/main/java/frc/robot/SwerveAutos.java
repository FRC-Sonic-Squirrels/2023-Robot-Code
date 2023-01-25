package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.FollowPathWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.List;

public class SwerveAutos {
  private Drivetrain drivetrain;
  private Intake intake;

  public SwerveAutos(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    DrivetrainConstants.EVENT_MAP.put(
        "extendIntake",
        Commands.runOnce(intake::extend, intake)
            .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.5), intake)));
    DrivetrainConstants.EVENT_MAP.put(
        "retractIntake",
        Commands.runOnce(intake::retract, intake)
            .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.0), intake)));
    DrivetrainConstants.EVENT_MAP.put(
        "scoreCube",
        new SequentialCommandGroup(new PrintCommand("cube scored"), new WaitCommand(1)));
      DrivetrainConstants.EVENT_MAP.put(
        "scoreCone",
        new SequentialCommandGroup(new PrintCommand("cone scored"), new WaitCommand(1)));
    DrivetrainConstants.EVENT_MAP.put(
        "groundPickup",
        new SequentialCommandGroup(new PrintCommand("object picked up"), new WaitCommand(1)));
    DrivetrainConstants.EVENT_MAP.put(
        "engage",
        new SequentialCommandGroup(new PrintCommand("engaged"), new WaitCommand(1)));
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

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "testPath2mForwardWithIntake",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
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
            AUTO_MAX_SPEED_METERS_PER_SECOND * 0.1,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command middle1Ball() {
    return new SequentialCommandGroup(
      DrivetrainConstants.EVENT_MAP.get("score"));
  }

  public Command middle1BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "middle1BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      middle1Ball(), 
      new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command right1Ball() {
    return new SequentialCommandGroup(
      DrivetrainConstants.EVENT_MAP.get("score"));
  }

  public Command right1BallTaxi() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right1BallTaxi",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      right1Ball(), 
      new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command right2Ball() {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right2Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      right1BallTaxi(),
        new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command right2BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right2BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      right2Ball(),
      new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command right3Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right3Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        right2Ball(),
        new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command left1Ball() {
    return new SequentialCommandGroup(
      DrivetrainConstants.EVENT_MAP.get("score")
    );
  }

  public Command left1BallTaxi() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left1BallTaxi",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      left1Ball(), 
      new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command left2Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left2Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left1BallTaxi(),
        new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command left2BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left2BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left2Ball(),
        new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }

  public Command left3Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left3Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
      left2Ball(),
      new FollowPathWithEvents(new FollowPath(path, drivetrain, true), path.getMarkers(), DrivetrainConstants.EVENT_MAP));
  }
}
