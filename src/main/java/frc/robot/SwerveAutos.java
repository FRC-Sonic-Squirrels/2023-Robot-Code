package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
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

  private void setInitialTrajectory(int hashCode, Trajectory trajectory) {
    trajectoryMap.put(hashCode, trajectory);
  }

  public Trajectory getInitialTrajectory(int hashCode) {
    return trajectoryMap.get(hashCode);
  }

  public AutoChooserElement testPath2mForward() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward",
            AUTO_TEST_MAX_SPEED_METERS_PER_SECOND,
            AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public Command testPath2mForward180() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward180",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c = new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
    setInitialTrajectory(c.hashCode(), path);

    return c;
  }

  public AutoChooserElement testPath3mForward360() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "3mForward360",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public Command curve() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "curve",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c = new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
    setInitialTrajectory(c.hashCode(), path);

    return c;
  }

  public Command rotateAroundPoint() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "rotateAroundPoint",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c = new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
    setInitialTrajectory(c.hashCode(), path);

    return c;
  }

  public Command testPath2mForwardWithIntake() {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "testPath2mForwardWithIntake",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c =
        new SequentialCommandGroup(
            new FollowPathWithEvents(
                new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));
    setInitialTrajectory(c.hashCode(), path);

    return c;
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

  public AutoChooserElement forwardLeft() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "forwardLeft",
            AUTO_MAX_SPEED_METERS_PER_SECOND * 0.1,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public SequentialCommandGroup middle1Ball() {
    return new SequentialCommandGroup(getEventMap().get("scoreCube"));
  }

  public Command middle1BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "middle1BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    SequentialCommandGroup c =
        new SequentialCommandGroup(
            middle1Ball(),
            new FollowPathWithEvents(
                new FollowPath(path, drivetrain, true), path.getMarkers(), getEventMap()));

    setInitialTrajectory(c.hashCode(), path);

    return c;
  }

  public AutoChooserElement right1Ball() {
    return new AutoChooserElement(
        null, () -> new SequentialCommandGroup(getEventMap().get("scoreCone")));
  }

  public AutoChooserElement right1BallTaxi() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right1BallTaxi",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return right1Ball().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement right2Ball() {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right2Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return right1BallTaxi()
        .setNext(
            new AutoChooserElement(
                path,
                () ->
                    new SequentialCommandGroup(
                        new FollowPathWithEvents(
                            new FollowPath(path, drivetrain, false),
                            path.getMarkers(),
                            getEventMap()))));
  }

  public AutoChooserElement right2BallEngage() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right2BallEngage",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return right2Ball()
        .setNext(
            new AutoChooserElement(
                path,
                () ->
                    new SequentialCommandGroup(
                        new FollowPathWithEvents(
                            new FollowPath(path, drivetrain, false),
                            path.getMarkers(),
                            getEventMap()))));
  }

  public AutoChooserElement right3Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right3Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return right2Ball()
        .setNext(
            new AutoChooserElement(
                path,
                () ->
                    new SequentialCommandGroup(
                        new FollowPathWithEvents(
                            new FollowPath(path, drivetrain, false),
                            path.getMarkers(),
                            getEventMap()))));
  }

  public AutoChooserElement right4Ball() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "right4Ball",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return right3Ball()
        .setNext(
            new AutoChooserElement(
                path,
                () ->
                    new SequentialCommandGroup(
                        new FollowPathWithEvents(
                            new FollowPath(path, drivetrain, false),
                            path.getMarkers(),
                            getEventMap()))));
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
    setInitialTrajectory(c.hashCode(), path);

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
