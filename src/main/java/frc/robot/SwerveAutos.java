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

    // System.out.println("TransformTrajectoryForAlliance: " + DriverStation.getAlliance().name());

    transformedTrajectory =
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            trajectory, DriverStation.getAlliance());

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
        name, AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
  }

  public AutoChooserElement doNothing() {
    return new AutoChooserElement(null, () -> new SequentialCommandGroup());
  }

  public AutoChooserElement testPath2mForward() {
    PathPlannerTrajectory path = loadPath("2mForward");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement testPath2mForward180() {
    PathPlannerTrajectory path = loadPath("2mForward180");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement testPath3mForward360() {
    PathPlannerTrajectory path = loadPath("3mForward360");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement curve() {
    PathPlannerTrajectory path = loadPath("curve");

    return new AutoChooserElement(
        path, () -> new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
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

  public AutoChooserElement scoreCone() {
    return new AutoChooserElement(
        null, () -> new SequentialCommandGroup(getEventMap().get("scoreCone")));
  }

  public AutoChooserElement scoreCube() {
    return new AutoChooserElement(
        null, () -> new SequentialCommandGroup(getEventMap().get("scoreCube")));
  }

  public AutoChooserElement middle1BallEngage() {
    PathPlannerTrajectory path = loadPath("middle1BallEngage");

    return scoreCube().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement right1BallTaxi() {
    PathPlannerTrajectory path = loadPath("right1BallTaxi");

    return scoreCone().setNext(path, true, drivetrain, getEventMap());
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

  public AutoChooserElement left1BallTaxi() {
    PathPlannerTrajectory path = loadPath("left1BallTaxi");

    return scoreCone().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement left2Ball() {
    PathPlannerTrajectory path = loadPath("left2Ball");

    return left1BallTaxi().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement left2BallEngage() {
    PathPlannerTrajectory path = loadPath("left2BallEngage");

    return left2Ball().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement left3Ball() {
    PathPlannerTrajectory path = loadPath("left3Ball");

    return left2Ball().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement left4Ball() {
    PathPlannerTrajectory path = loadPath("left4Ball");

    return left3Ball().setNext(path, true, drivetrain, getEventMap());
  }
}
