package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPath;
import frc.robot.commands.FollowPathWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import java.util.HashMap;
import java.util.List;

public class SwerveAutos {
  private Drivetrain drivetrain;
  private Intake intake;
  HashMap<Integer, Trajectory> trajectoryMap = new HashMap<>();

  public SwerveAutos(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  private HashMap<String, Command> getEventMap() {
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put(
        "extendIntake",
        Commands.runOnce(intake::extend, intake)
            .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.5), intake)));
    eventMap.put(
        "retractIntake",
        Commands.runOnce(intake::retract, intake)
            .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.0), intake)));
    eventMap.put(
        "scoreCube",
        new SequentialCommandGroup(new PrintCommand("cube scored"), Commands.waitSeconds(2)));
    eventMap.put(
        "scoreCone",
        new SequentialCommandGroup(new PrintCommand("cone scored"), Commands.waitSeconds(2)));
    eventMap.put(
        "groundPickup",
        new SequentialCommandGroup(new PrintCommand("object picked up"), Commands.waitSeconds(2)));
    eventMap.put(
        "engage", new SequentialCommandGroup(new PrintCommand("engaged"), Commands.waitSeconds(2)));

    return eventMap;
  }

  /**
   * loadPath - Load a PathPlanner path file and create a PathPlannerTrajectory object.
   *
   * @param name name of the PathPlanner path to load the file
   * @param maxVelocity max robot velocity in m/s
   * @param maxAcceleration max robot acceleration in m/s^2
   * @return path
   */
  public PathPlannerTrajectory loadPath(String name, double maxVelocity, double maxAcceleration) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVelocity, maxAcceleration);
    PathPlannerTrajectory transformedTrajectory;

    if (trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    return transformedTrajectory;
  }

  /**
   * loadPath - Load a PathPlanner path file and create a PathPlannerTrajectory object. Uses the
   * default autonomous velocity and acceleration.
   *
   * @param name name of the PathPlanner path to load the file
   * @return path
   */
  public PathPlannerTrajectory loadPath(String name) {
    return loadPath(
        name,
        AUTO_TEST_MAX_SPEED_METERS_PER_SECOND,
        AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
  }

  public AutoChooserElement testPath2mForward() {
    PathPlannerTrajectory path = loadPath("2mForward");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public Command testPath2mForward180() {
    PathPlannerTrajectory path = loadPath("2mForward180");

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public AutoChooserElement testPath3mForward360() {
    PathPlannerTrajectory path = loadPath("3mForward360");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public Command curve() {
    PathPlannerTrajectory path = loadPath("curve");

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command rotateAroundPoint() {
    PathPlannerTrajectory path = loadPath("rotateAroundPoint");

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath2mForwardWithIntake() {
    PathPlannerTrajectory path = loadPath("testPath2mForwardWithIntake");

    SequentialCommandGroup c =
        new SequentialCommandGroup(
            new FollowPathWithEvents(
                new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));

    return c;
  }

  // FIXME: no support yet for loadPathGroup
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

  public AutoChooserElement forwardLeft() {
    PathPlannerTrajectory path = loadPath("forwardLeft");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public SequentialCommandGroup middle1Ball() {
    return new SequentialCommandGroup(getEventMap().get("scoreCube"));
  }

  public Command middle1BallEngage() {
    PathPlannerTrajectory path = loadPath("middle1BallEngage");

    SequentialCommandGroup c =
        new SequentialCommandGroup(
            middle1Ball(),
            new FollowPathWithEvents(
                new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));

    return c;
  }

  public AutoChooserElement right1Ball() {
    return new AutoChooserElement(
        null, () -> new SequentialCommandGroup(getEventMap().get("scoreCone")));
  }

  public AutoChooserElement right1BallTaxi() {
    PathPlannerTrajectory path = loadPath("right1BallTaxi");

    return right1Ball().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement right2Ball() {
    PathPlannerTrajectory path = loadPath("right2Ball");

    return right1BallTaxi().setNext(path, false, drivetrain, getEventMap());
  }

  public AutoChooserElement right2BallEngage() {
    PathPlannerTrajectory path = loadPath("right2BallEngage");

    return right2Ball().setNext(path, false, drivetrain, getEventMap());
  }

  public AutoChooserElement right3Ball() {
    PathPlannerTrajectory path = loadPath("right3Ball");

    return right2Ball().setNext(path, false, drivetrain, getEventMap());
  }

  public AutoChooserElement right4Ball() {
    PathPlannerTrajectory path = loadPath("right4Ball");

    return right3Ball().setNext(path, false, drivetrain, getEventMap());
  }

  public Command left1Ball() {
    return new SequentialCommandGroup(getEventMap().get("scoreCone"));
  }

  public Command left1BallTaxi() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left1BallTaxi",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c =
        new SequentialCommandGroup(
            left1Ball(),
            new FollowPathWithEvents(
                new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));

    return c;
  }

  public Command left2Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left2Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left1BallTaxi(),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));
  }

  public Command left2BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left2BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left2Ball(),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));
  }

  public Command left3Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left3Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left2Ball(),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));
  }

  public Command left4Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "left4Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
        left3Ball(),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));
  }
}
