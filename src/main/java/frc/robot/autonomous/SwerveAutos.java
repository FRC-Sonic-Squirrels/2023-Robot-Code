package frc.robot.autonomous;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team2930.AutoChooserElement;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SwerveAutos {
  private HashMap<String, Supplier<AutoChooserElement>> autonomousCommands = new HashMap<>();
  private List<String> names = new ArrayList<>();
  private Drivetrain drivetrain;
  private Intake intake;
  private Elevator elevator;
  private Stinger stinger;
  // private LED led;

  // get field dimensions from official WPILib file
  // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2023-chargedup.json#L148-L151
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  public SwerveAutos(Drivetrain drivetrain, Intake intake, Elevator elevator, Stinger stinger) {
    // FIXME: List of all required subsystems: elevator, stinger, intake, LED
    // iterate through all the commands in EventMap and extract the requirements for each command?
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.elevator = elevator;
    this.stinger = stinger;
    // this.led = led;

    //
    // Add named commands here.
    //

    // FIXME: temporarily make this the default command for testing vision
    // addCommand("left1BallTaxi", () -> left1BallTaxi());

    // NOTE: doNothing command needs to be first so that it is always the default in chooser
    addCommand("Do Nothing", () -> doNothing());

    if (!DriverStation.isFMSAttached()) {
      // only add test paths when not connected to FMS
      addCommand("Test Sequential", () -> testSeq());
      addCommand("2m Forward", () -> testPath2mForward());
      addCommand("2m Forward w/ 180", () -> testPath2mForward180());
      addCommand("3m Forward 2/ 360", () -> testPath3mForward360());
      addCommand("forwardLeft", () -> forwardLeft());
    }
    addCommand("scoreCone", () -> scoreConeHigh());
    addCommand("scoreCube", () -> scoreCubeHigh());
    addCommand("middle1BallEngage", () -> middle1BallEngage());
    addCommand("right1BallTaxi", () -> right1BallTaxi());
    addCommand("right2Ball", () -> right2Ball());
    addCommand("right2BallEngage", () -> right2BallEngage());
    // addCommand("right3Ball", () -> right3Ball());
    // addCommand("right4Ball", () -> right4Ball());
    addCommand("left1BallTaxi", () -> left1BallTaxi());
    addCommand("left2Ball", () -> left2Ball());
    addCommand("left2BallEngage", () -> left2BallEngage());
    // addCommand("left3Ball", () -> left3Ball());
    // addCommand("left4Ball", () -> left4Ball());
  }

  /**
   * addCommand() - add a new autonomous command for the chooser
   *
   * @param name the name of the autonomous command displayed in the chooser
   * @param command autonomous command
   */
  private void addCommand(String name, Supplier<AutoChooserElement> command) {
    autonomousCommands.put(name, command);
    // create a list of names to preserver the order
    names.add(name);
  }

  /**
   * getAutonomousCommandNames - return list of autonomous routine names
   *
   * @return list of names of all the autonomous routines
   */
  public List<String> getAutonomousCommandNames() {
    return names;
  }

  /**
   * getChooserElement - return a supplier that will return the chooser element object.
   *
   * <p>A supplier is returned, and not the actual object to avoid generating the associated
   * trajectories and commands until the element is actually needed.
   *
   * @param name the chooser name string
   * @return AutoChooserElement supplier
   */
  public Supplier<AutoChooserElement> getChooserElement(String name) {
    if (autonomousCommands.containsKey(name)) {
      return autonomousCommands.get(name);
    }

    // else the key doesn't exist, throw an error
    System.out.println("ERROR: no such autonomous [" + name + "]");

    return (() -> doNothing());
  }

  /**
   * eventMap() - generate a fresh PathPlanner EventMap
   *
   * @return EventMap
   */
  private HashMap<String, Command> getEventMap() {
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("extendIntake", Commands.runOnce(() -> intake.runIntakePercent(0.5), intake));
    eventMap.put("retractIntake", Commands.runOnce(() -> intake.runIntakePercent(0.0), intake));
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
    eventMap.put(
        "test",
        new SequentialCommandGroup(
            new PrintCommand("test1 1"),
            Commands.waitSeconds(0.1),
            new PrintCommand("test1 2"),
            Commands.waitSeconds(0.1),
            new PrintCommand("test1 3")));
    eventMap.put(
        "test2",
        new SequentialCommandGroup(
            new PrintCommand("test2 1"),
            Commands.waitSeconds(0.1),
            new PrintCommand("test2 2"),
            Commands.waitSeconds(0.1),
            new PrintCommand("test2 3")));
    eventMap.put(
        "slowTest",
        new SequentialCommandGroup(
            new PrintCommand("slow 1"),
            Commands.waitSeconds(0.5),
            new PrintCommand("slow 2"),
            Commands.waitSeconds(0.5),
            new PrintCommand("slow 3")));

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

    // System.out.println(
    //     "TransformTrajectoryForAlliance: " + DriverStation.getAlliance().name() + " path: " +
    // name);

    var alliance = DriverStation.getAlliance();
    if ((trajectory != null) && (alliance == DriverStation.Alliance.Red)) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);

      for (var i = 0; i < trajectory.getStates().size(); i++) {
        var oldState = (PathPlannerTrajectory.PathPlannerState) trajectory.getState(i);
        var newState = (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.getState(i);

        Pose2d pose2d = oldState.poseMeters;
        var x = pose2d.getX();
        var y = pose2d.getY();
        var rot = pose2d.getRotation();
        rot = new Rotation2d(-rot.getCos(), rot.getSin());

        Rotation2d holonomicRotation = oldState.holonomicRotation;
        newState.holonomicRotation =
            new Rotation2d(-holonomicRotation.getCos(), holonomicRotation.getSin());

        newState.poseMeters = new Pose2d(FIELD_LENGTH_METERS - x, y, rot);
      }

      return transformedTrajectory;
    }

    return trajectory;
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

  // ========================= TEST Autonomous Routines ========================

  public AutoChooserElement doNothing() {
    return new AutoChooserElement(null, new SequentialCommandGroup());
  }

  public AutoChooserElement testPath2mForward() {
    PathPlannerTrajectory path = loadPath("2mForward");

    return new AutoChooserElement(
        path, new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement testPath2mForward180() {
    PathPlannerTrajectory path = loadPath("2mForward180");

    return new AutoChooserElement(
        path, new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement testPath3mForward360() {
    PathPlannerTrajectory path = loadPath("3mForward360");

    return new AutoChooserElement(
        path, new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement curve() {
    PathPlannerTrajectory path = loadPath("curve");

    return new AutoChooserElement(
        path, new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
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
        path, new SequentialCommandGroup(new FollowPath(path, drivetrain, true)));
  }

  public AutoChooserElement testSeq() {
    PathPlannerTrajectory path = loadPath("2mforward");

    return doNothing().setNext(path, true, drivetrain, getEventMap()).setNext(scoreConeHigh());
  }

  // ======================= COMPETITION Autonomous Routines ========================
  //
  // Commands are built on preceding commands and are constructed using function
  // composition. For most commands, the new command is the old command composed
  // with a new trajectory. Actions are performed from the supplied EventMap and
  // triggered by event markers set with PathPlanner.
  //
  public AutoChooserElement scoreConeHigh() {
    return new AutoChooserElement(
        null,
        new SequentialCommandGroup(
            MechanismPositions.scoreConeHighPosition(elevator, stinger, intake)));
  }

  public AutoChooserElement scoreCubeHigh() {
    return new AutoChooserElement(
        null,
        new SequentialCommandGroup(
            MechanismPositions.scoreCubeHighPosition(elevator, stinger, intake)));
  }

  public AutoChooserElement scoreConeMid() {
    return new AutoChooserElement(
        null,
        new SequentialCommandGroup(
            MechanismPositions.scoreCubeMidPosition(elevator, stinger, intake)));
  }

  public AutoChooserElement scoreCubeMid() {
    return new AutoChooserElement(
        null,
        new SequentialCommandGroup(
            MechanismPositions.scoreConeMidPosition(elevator, stinger, intake),
            Commands.waitUntil(mechanismInPos()),
            new IntakeScoreCube(intake).raceWith(new WaitCommand(0.5)),
            MechanismPositions.stowPosition(elevator, stinger)));
  }

  private BooleanSupplier mechanismInPos() {
    return () -> (elevator.isAtHeight() && stinger.isAtExtension());
  }

  public AutoChooserElement middle1BallEngage() {
    PathPlannerTrajectory path = loadPath("middle1BallEngage");

    return scoreCubeHigh().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement right1BallTaxi() {
    PathPlannerTrajectory path = loadPath("right1BallTaxi");

    return scoreConeHigh().setNext(path, true, drivetrain, getEventMap());
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
    PathPlannerTrajectory path = loadPath("Left1BallTaxi");

    return scoreConeHigh().setNext(path, true, drivetrain, getEventMap());
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
