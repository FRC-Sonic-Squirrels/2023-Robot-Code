package frc.robot.autonomous;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AutoChooserElement;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.drive.AutoEngage;
import frc.robot.commands.drive.DriveWithSetRotation;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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
    // addCommand("Hp1BallTaxi", () -> Hp1BallTaxi());

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
    addCommand("middle1PieceEngage", () -> middle1BallEngage());
    addCommand("middleDriveOutEngage", () -> middleDriveOutAndEngage());
    // addCommand("middle2PieceEngage", () -> middle2BallEngage());
    addCommand("Wall1PieceTaxi", () -> wall1BallTaxi());
    addCommand("Wall2Piece", () -> wall2Ball());
    // addCommand("Wall2PieceEngage", () -> wall2BallEngage());
    // addCommand("Wall3Piece", () -> wall3Ball());
    // addCommand("Wall4Piece", () -> wall4Ball());
    addCommand("Hp1PieceTaxi", () -> Hp1BallTaxi());
    addCommand("Hp2Piece", () -> Hp2Ball());
    // addCommand("Hp2PieceEngage", () -> Hp2BallEngage());
    // addCommand("Hp3Piece", () -> Hp3Ball());
    // addCommand("Hp4Piece", () -> Hp4Ball());

    // addCommand(
    //     "driveAutoEngage", () -> driveAutoEngage(DriverStation.getAlliance() == Alliance.Red));
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
        "mechIntakeCube",
        new ParallelCommandGroup(
            MechanismPositions.groundPickupPosition(elevator, stinger),
            new IntakeGrabCube(intake).withTimeout(4)));
    eventMap.put("mechStow", MechanismPositions.stowPosition(elevator, stinger));
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
    PathPlannerTrajectory path = loadPath("2mForward");

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
            MechanismPositions.scoreCubeMidPosition(elevator, stinger, intake)));
  }

  public AutoChooserElement driveAutoEngage(Boolean flip) {
    return new AutoChooserElement(
        null,
        new SequentialCommandGroup(
            new ConditionalCommand(
                new DriveWithSetRotation(drivetrain, elevator, stinger, () -> (-0.7), () -> 0, 180)
                    .until(() -> drivetrain.getGyroPitch() <= -15),
                new DriveWithSetRotation(drivetrain, elevator, stinger, () -> 0.7, () -> 0, 0)
                    .until(() -> drivetrain.getGyroPitch() >= 15),
                () -> flip),
            new AutoEngage(drivetrain, !flip)));
  }

  public AutoChooserElement middle1BallEngage() {
    // PathPlannerTrajectory path = loadPath("middle1PieceEngage");

    return scoreConeHigh().setNext(driveAutoEngage(DriverStation.getAlliance() == Alliance.Red));
  }

  public AutoChooserElement middleDriveOutAndEngage() {

    boolean onRed = DriverStation.getAlliance() == Alliance.Red;
    double forward = onRed ? -2.0 : 2.0;

    double rotation = onRed ? 0.0 : 180;

    boolean flipAutoEngage = onRed ? true : false;

    Pose2d startPose =
        onRed
            ? new Pose2d(14.69, 3.29, Rotation2d.fromDegrees(0))
            : new Pose2d(1.88, 2.18, Rotation2d.fromDegrees(180));

    return scoreConeHigh()
        .setNext(
            new SequentialCommandGroup(
                Commands.runOnce(() -> drivetrain.resetOdometry(startPose), drivetrain),
                Commands.runEnd(
                        () -> drivetrain.drive(forward, 0.0, 0.0),
                        () -> drivetrain.stop(),
                        drivetrain)
                    .until(() -> Math.abs(drivetrain.getGyroPitch()) > 15)
                    .withTimeout(2),
                // --
                Commands.runEnd(
                        () -> drivetrain.drive(forward, 0.0, 0.0),
                        () -> drivetrain.stop(),
                        drivetrain)
                    .until(
                        new Trigger(() -> Math.abs(drivetrain.getGyroPitch()) < 3).debounce(0.45))
                    .withTimeout(2.25),
                // --

                // new DriveWithSetRotation(drivetrain, () -> 0.0, () -> 0.0, (int) rotation)
                //     .withTimeout(1.5),
                // --
                Commands.run(() -> drivetrain.drive(-forward, 0.0, 0.0), drivetrain)
                    .until(() -> Math.abs(drivetrain.getGyroPitch()) > 15)
                    .withTimeout(2.0),
                new AutoEngage(drivetrain, flipAutoEngage)
                    .handleInterrupt(() -> drivetrain.enableXstance())));
  }

  // public AutoChooserElement middle2BallEngage() {
  //   PathPlannerTrajectory path = loadPath("middle2PieceEngage");

  //   return scoreCubeHigh()
  //       .setNext(path, true, drivetrain, getEventMap())
  //       .setNext(driveAutoEngage(DriverStation.getAlliance() == Alliance.Red));
  // }

  public AutoChooserElement wall1BallTaxi() {
    PathPlannerTrajectory path = loadPath("wallSide1PieceTaxi");

    return scoreConeHigh().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement wall2Ball() {
    PathPlannerTrajectory path = loadPath("wallSide2Piece");

    return wall1BallTaxi().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeHigh());
  }

  public AutoChooserElement wall2BallEngage() {
    PathPlannerTrajectory path = loadPath("wallSide2PieceEngage");

    return wall2Ball()
        .setNext(path, false, drivetrain, getEventMap())
        .setNext(driveAutoEngage(DriverStation.getAlliance() == Alliance.Blue));
  }

  public AutoChooserElement wall3Ball() {
    PathPlannerTrajectory path = loadPath("wallSide3Piece");

    return wall2Ball().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeMid());
  }

  public AutoChooserElement wall4Ball() {
    PathPlannerTrajectory path = loadPath("wallSide4Piece");

    return wall3Ball().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeHigh());
  }

  public AutoChooserElement Hp1BallTaxi() {
    PathPlannerTrajectory path = loadPath("HpSide1PieceTaxi");

    return scoreConeHigh().setNext(path, true, drivetrain, getEventMap());
  }

  public AutoChooserElement Hp2Ball() {
    PathPlannerTrajectory path = loadPath("HpSide2Piece");

    return Hp1BallTaxi().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeHigh());
  }

  public AutoChooserElement Hp2BallEngage() {
    PathPlannerTrajectory path = loadPath("HpSide2PieceEngage");

    return Hp2Ball()
        .setNext(path, false, drivetrain, getEventMap())
        .setNext(driveAutoEngage(DriverStation.getAlliance() == Alliance.Blue));
  }

  public AutoChooserElement Hp3Ball() {
    PathPlannerTrajectory path = loadPath("HpSide3Piece");

    return Hp2Ball().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeMid());
  }

  public AutoChooserElement Hp4Ball() {
    PathPlannerTrajectory path = loadPath("HpSide4Piece");

    return Hp3Ball().setNext(path, false, drivetrain, getEventMap()).setNext(scoreCubeHigh());
  }
}
