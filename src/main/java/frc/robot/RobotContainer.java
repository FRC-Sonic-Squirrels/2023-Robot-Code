// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.PIGEON_CAN_BUS_NAME;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.PIGEON_ID;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AutoChooserElement;
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.HumanLoadingStationHandler;
import frc.lib.team2930.driverassist.HumanLoadingStationHandler.LoadingStationLocation;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIOConfig;
import frc.lib.team3061.vision.VisionIOSim;
import frc.lib.team3061.vision.VisionNew;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.GamePiece;
import frc.robot.autonomous.SwerveAutos;
import frc.robot.commands.drive.DriveWithSetRotation;
import frc.robot.commands.drive.SnapToGrid;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.commands.intake.IntakeAutoGrabDesiredGamePiece;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.leds.LedSetColor;
import frc.robot.commands.leds.LedSetColorNoEnd;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.commands.stinger.StingerManualControl;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal2023;
import frc.robot.subsystems.elevator.ElevatorSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIO2023;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.colors;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.stinger.Stinger;
import frc.robot.subsystems.stinger.StingerIO;
import frc.robot.subsystems.stinger.StingerIOReal;
import frc.robot.subsystems.stinger.StingerSim;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  //   private final CommandXboxController driverAssistController = new CommandXboxController(2);
  /* Driver Buttons */
  // these triggers are now directly detected
  // zeroGyro is assigned to back
  // robotCentric is assigned to b
  // xStance is assigned to a
  // intakeOut is assigned to right bumper

  private Drivetrain drivetrain;
  private Intake intake;
  public SwerveAutos autos;
  private Stinger stinger;
  private Elevator elevator;
  private LED leds;
  public VisionNew vision;

  private DriverAssistAutos driverAssist;
  public final GridPositionHandler gridPositionHandler = GridPositionHandler.getInstance();

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private LoggedDashboardChooser<Supplier<AutoChooserElement>> autoChooser;

  private TunableNumber yeetHeight = new TunableNumber("yeet/elevatorHeightTeleop", 35);
  private TunableNumber yeetExtensionThreshold =
      new TunableNumber("yeet/stingerExtensionThresholdTeleop", 15);

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023_PRESEASON:
          {
            GyroIO gyro = new GyroIOPigeon2(PIGEON_ID, PIGEON_CAN_BUS_NAME);

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            leds = new LED(new LEDIO() {});

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            // new Vision(VisionConstants.LEFT_ROBOT_TO_CAMERA, new
            // VisionIOPhotonVision(CAMERA_NAME));
            // intake = new Intake(new IntakeIOFalcon());
            // elevator = new Elevator(new ElevatorReal2022());

            // wrist = new Wrist(new WristIOSolenoid());
            break;
          }
        case ROBOT_2023_COMPBOT:
          {
            GyroIO gyro = new GyroIOPigeon2(PIGEON_ID, PIGEON_CAN_BUS_NAME);

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            elevator = new Elevator(new ElevatorReal2023());
            stinger = new Stinger(new StingerIOReal());
            intake = new Intake(new IntakeIO2023());
            leds = new LED(new LEDIOReal());

            // vision =
            //     new Vision(
            //         new VisionIOPhotonVision(Constants.LEFT_CAMERA_NAME),
            //         new VisionIOPhotonVision(Constants.RIGHT_CAMERA_NAME),
            //         new VisionIOPhotonVision(Constants.BACK_CAMERA_NAME),
            //         drivetrain);

            vision = null;

            RobotState.getInstance().setDesiredGamePiece(GamePiece.CONE);
            leds.setColor(colors.YELLOW);

            break;
          }

        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            AprilTagFieldLayout layout;
            try {
              layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            // vision =
            //     new Vision(
            //         new VisionIOSim(
            //             layout,
            //             drivetrain::getPose,
            //             VisionConstants.LEFT_ROBOT_TO_CAMERA,
            //             "leftCameraNetwork"),
            //         new VisionIOSim(
            //             layout,
            //             drivetrain::getPose,
            //             VisionConstants.RIGHT_ROBOT_TO_CAMERA,
            //             "rightCameraNetwork"),
            //         new VisionIOSim(
            //             layout,
            //             drivetrain::getPose,
            //             VisionConstants.BACK_ROBOT_TO_CAMERA,
            //             "backCameraNetwork"),
            //         drivetrain);

            VisionIOConfig frontLeftConfig =
                new VisionIOConfig(
                    new VisionIOSim(
                        layout,
                        drivetrain::getPose,
                        VisionConstants.LEFT_ROBOT_TO_CAMERA,
                        "leftCameraNetwork"),
                    "frontLeft",
                    VisionConstants.LEFT_ROBOT_TO_CAMERA);

            VisionIOConfig frontRightConfig =
                new VisionIOConfig(
                    new VisionIOSim(
                        layout,
                        drivetrain::getPose,
                        VisionConstants.RIGHT_ROBOT_TO_CAMERA,
                        "rightCameraNetwork"),
                    "frontRight",
                    VisionConstants.RIGHT_ROBOT_TO_CAMERA);

            VisionIOConfig backConfig =
                new VisionIOConfig(
                    new VisionIOSim(
                        layout,
                        drivetrain::getPose,
                        VisionConstants.BACK_ROBOT_TO_CAMERA,
                        "backCameraNetwork"),
                    "back",
                    VisionConstants.BACK_ROBOT_TO_CAMERA);

            vision = new VisionNew(drivetrain, frontLeftConfig, frontRightConfig, backConfig);

            leds = new LED(new LEDIO() {});
            intake = new Intake(new IntakeIO() {});
            elevator = new Elevator(new ElevatorSim());
            stinger = new Stinger(new StingerSim());

            DriverStation.silenceJoystickConnectionWarning(true);
            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      elevator = new Elevator(new ElevatorIO() {});
      stinger = new Stinger(new StingerIO() {});
      intake = new Intake(new IntakeIO() {});
      leds = new LED(new LEDIO() {});
      //   vision = new Vision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, drivetrain);
      vision = null;
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */

    driverAssist =
        new DriverAssistAutos(drivetrain, intake, elevator, stinger, leds, driverController);

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            elevator,
            stinger,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX));

    intake.setDefaultCommand(
        new IntakeAutoGrabDesiredGamePiece(intake).withTimeout(0.15).repeatedly());

    leds.setDefaultCommand(
        new ConditionalCommand(
                new LedSetColor(leds, colors.YELLOW),
                new LedSetColor(leds, colors.VIOLET),
                () -> RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)
            .repeatedly());

    // FIX ME: these default commands cause all sorts of headaches when trying to use closed loop
    // positional control
    // instead of working around it lets just have a button.whileTrue where these are only active if
    // that button is pressed.
    // if we do this system remember to make the end stop the system incase we let go of the while
    // true button when joystick has a value > 0.0
    // otherwise the system will run at that percent speed until it hits soft/hard limit.

    configureButtonBindings();
    configureAutoCommands();

    HumanLoadingStationHandler.logAllHumanLoadingStationDriverAssist();
    GridPositionHandler.logAllGridPositionDriverAssist();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    // Debug Swerve Commands
    // useful don't delete these
    // driverController.a().whileTrue(Commands.run(() -> drivetrain.drive(-1, 0, 0), drivetrain));
    // driverController.b().whileTrue(Commands.run(() -> drivetrain.drive(0, -1, 0), drivetrain));
    // driverController.x().whileTrue(Commands.run(() -> drivetrain.drive(0, 1, 0), drivetrain));
    // driverController.y().whileTrue(Commands.run(() -> drivetrain.drive(1, 0, 0), drivetrain));

    // FIXME: UNCOMMENT button bindings

    // toggle between Field and Robot centric driving
    // we should not ever need robot relative in this game
    // driverController
    //     .b()
    //     .toggleOnTrue(
    //         Commands.either(
    //             Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    //             Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
    //             drivetrain::getFieldRelative));

    // FIXME: this should be put into 3rd controller or operator controller
    // driverController
    //     .y()
    //     .onTrue(Commands.runOnce(() -> vision.disableMaxDistanceAwayForTags(), vision))
    //     .onFalse(Commands.runOnce(() -> vision.enableMaxDistanceAwayForTags(), vision));

    // reset if weird behavior button
    driverController
        .a()
        .onTrue(drivetrain.getDefaultCommand())
        .onTrue(Commands.runOnce(drivetrain::enableFieldRelative))
        .onTrue(Commands.runOnce(drivetrain::disableXstance));

    // Certified oh crap button
    // driverController.b().onTrue(MechanismPositions.safeStowPosition(elevator, stinger));

    // driverController.y().onTrue(MechanismPositions.stowPosition(elevator, stinger));
    // // reset gyro to 0 degrees
    driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    driverController
        .start()
        .onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain))
        .onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    driverController
        .povDown()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    elevator,
                    stinger,
                    driverController::getLeftY,
                    driverController::getLeftX,
                    180)
                .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    driverController
        .povUp()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    elevator,
                    stinger,
                    driverController::getLeftY,
                    driverController::getLeftX,
                    0)
                .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    driverController
        .povRight()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    elevator,
                    stinger,
                    driverController::getLeftY,
                    driverController::getLeftX,
                    -30.0)
                .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    driverController
        .povLeft()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    elevator,
                    stinger,
                    driverController::getLeftY,
                    driverController::getLeftX,
                    22)
                .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    // TODO: test this to see if it works
    // driverController
    //     .x()
    //     .onTrue(
    //         MechanismPositions.groundPickupPosition(elevator, stinger)
    //             .alongWith(new IntakeGrabCube(intake))
    //             .until(() -> intake.isStalled())
    //             .andThen(
    //                 MechanismPositions.stowPosition(elevator, stinger)
    //                     .alongWith(new LedSetColor(leds, colors.BLUE_STROBE))));

    operatorController
        .leftTrigger(0.75)
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorManualControl(elevator, () -> -operatorController.getLeftY()),
                new StingerManualControl(stinger, elevator, operatorController::getRightX)));

    // operatorController
    //     .x()
    //     .onTrue(
    //         MechanismPositions.groundPickupPosition(elevator, stinger)
    //             .alongWith(
    //                 Commands.runOnce(
    //                     () -> RobotState.getInstance().setDesiredGamePiece(GamePiece.CUBE)))
    //             .andThen(Commands.waitUntil(new Trigger(() ->
    // intake.isStalled()).debounce(0.05)))
    //             .deadlineWith(new IntakeGrabCube(intake))
    //             .andThen(
    //                 MechanismPositions.stowPosition(elevator, stinger)
    //                     .deadlineWith(new LedSetColorNoEnd(leds,
    // colors.BLUE_STROBE).asProxy())));

    operatorController
        .x()
        .onTrue(
            new ConditionalCommand(
                MechanismPositions.groundPickupPosition(elevator, stinger)
                    .andThen(
                        Commands.waitUntil(new Trigger(() -> intake.isStalled()).debounce(0.05)))
                    .deadlineWith(new IntakeGrabCube(intake))
                    .andThen(
                        MechanismPositions.stowPosition(elevator, stinger)
                            .deadlineWith(
                                new LedSetColorNoEnd(leds, colors.BLUE_STROBE).asProxy())),
                MechanismPositions.groundPickupPositionConeTeleop(elevator, stinger)
                    .andThen(
                        Commands.waitUntil(new Trigger(() -> intake.isStalled()).debounce(0.05)))
                    .deadlineWith(new IntakeGrabCone(intake))
                    .andThen(
                        MechanismPositions.stowPosition(elevator, stinger)
                            .deadlineWith(
                                new LedSetColorNoEnd(leds, colors.BLUE_STROBE).asProxy())),
                () -> RobotState.getInstance().getDesiredGamePiece() == GamePiece.CUBE));

    operatorController.b().onTrue(MechanismPositions.stowPosition(elevator, stinger));

    operatorController.a().onTrue(MechanismPositions.safeZero(elevator, stinger));

    operatorController
        .y()
        .onTrue(
            new ConditionalCommand(
                MechanismPositions.substationPickupPositionCone(elevator, stinger, intake),
                MechanismPositions.substationPickupPositionCube(elevator, stinger, intake),
                () -> RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE));

    // operatorController
    //     .y()
    //     .onTrue(
    //         MechanismPositions.substationPickupPositionCone(elevator, stinger, intake)
    //             .alongWith(
    //                 Commands.runOnce(
    //                     () -> RobotState.getInstance().setDesiredGamePiece(GamePiece.CONE)))
    //             .alongWith(new IntakeGrabCone(intake))
    //             .alongWith(
    //                 new ConditionalCommand(
    //                         new LedSetColorNoEnd(leds, colors.YELLOW_STROBE).asProxy(),
    //                         new InstantCommand(),
    //                         () -> intake.isStalledForCone())
    //                     .repeatedly()));

    operatorController
        .povUp()
        .onTrue(
            new ConditionalCommand(
                MechanismPositions.scoreConeHigh(elevator, stinger, intake, operatorController),
                MechanismPositions.scoreCubeHigh(elevator, stinger, intake, operatorController),
                () -> (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)));
    operatorController
        .povLeft()
        .onTrue(
            new ConditionalCommand(
                MechanismPositions.scoreConeMid(elevator, stinger, intake, operatorController),
                MechanismPositions.scoreCubeMid(elevator, stinger, intake, operatorController),
                () -> (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)));
    operatorController
        .povDown()
        .onTrue(
            new ConditionalCommand(
                MechanismPositions.scoreLow(
                    elevator, stinger, intake, GamePiece.CONE, operatorController),
                MechanismPositions.scoreLow(
                    elevator, stinger, intake, GamePiece.CUBE, operatorController),
                () -> (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)));

    operatorController
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                new IntakeGrabCone(intake, 1.0),
                new IntakeGrabCube(intake, 0.4),
                () -> (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)));
    operatorController
        .leftBumper()
        .whileTrue(
            new ConditionalCommand(
                new IntakeScoreCone(intake, 0.8),
                new IntakeScoreCube(intake, 0.5),
                () -> (RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE)));
    operatorController
        .back()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().setDesiredGamePiece(GamePiece.CUBE)));
    operatorController
        .start()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().setDesiredGamePiece(GamePiece.CONE)));

    // TODO: FIXME
    // operatorController
    //     .povRight()
    //     .onTrue(Commands.run(() -> vision.disableMaxDistanceAwayForTags(), vision))
    //     .onFalse(Commands.run(() -> vision.enableMaxDistanceAwayForTags(), vision));

    // driverController
    //     .leftTrigger(0.8)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               var scoringSequence = driverAssist.getScoringSequenceForGridPositionAuto();

    //               var cmd =
    //                   driverAssist.driveToLogicalBaySpecificEntrance(
    //                       RobotState.getInstance().getDesiredLogicalGrid(),
    //                       DesiredGridEntrance.LEFT,
    //                       scoringSequence); // some command

    //               Command currentCmd = drivetrain.getCurrentCommand();

    //               if (currentCmd instanceof OverrideDrivetrainStop) {
    //                 ((OverrideDrivetrainStop) currentCmd).overideStop();
    //               }

    //               cmd.schedule();
    //             }));

    // driverController
    //     .rightTrigger(0.8)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               var scoringSequence = driverAssist.getScoringSequenceForGridPositionAuto();

    //               var cmd =
    //                   driverAssist.driveToLogicalBaySpecificEntrance(
    //                       RobotState.getInstance().getDesiredLogicalGrid(),
    //                       DesiredGridEntrance.RIGHT,
    //                       scoringSequence); // some command

    //               Command currentCmd = drivetrain.getCurrentCommand();

    //               if (currentCmd instanceof OverrideDrivetrainStop) {
    //                 ((OverrideDrivetrainStop) currentCmd).overideStop();
    //               }

    //               cmd.schedule();
    //             }));

    // Post Season reflection (PSR) it would have been better to use a command that has a
    // Supplier<Trajectory> parameter instead of having to do weirdness with creating new command
    // objects every time
    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/DriveTrajectory.java
    driverController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // make intake spin
                  var pickUpSequence = driverAssist.getPickUpSequenceForHumanPlayerStation();
                  // stop intake spinning
                  var retractSequence = driverAssist.getRetractSequenceForHumanPlayerStation();

                  var cmd =
                      driverAssist.humanPlayerStation(
                          LoadingStationLocation.LEFT, pickUpSequence, retractSequence);

                  Command currentCmd = drivetrain.getCurrentCommand();

                  if (currentCmd instanceof OverrideDrivetrainStop) {
                    ((OverrideDrivetrainStop) currentCmd).overideStop();
                  }

                  cmd.schedule();
                }));

    driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // make intake spin
                  var pickUpSequence = driverAssist.getPickUpSequenceForHumanPlayerStation();
                  // stop intake spinning
                  var retractSequence = driverAssist.getRetractSequenceForHumanPlayerStation();

                  var cmd =
                      driverAssist.humanPlayerStation(
                          LoadingStationLocation.RIGHT, pickUpSequence, retractSequence);

                  Command currentCmd = drivetrain.getCurrentCommand();

                  if (currentCmd instanceof OverrideDrivetrainStop) {
                    ((OverrideDrivetrainStop) currentCmd).overideStop();
                  }

                  cmd.schedule();
                }));

    driverController
        .leftTrigger(0.5)
        .whileTrue(
            new SnapToGrid(drivetrain)
                .deadlineWith(new LedSetColorNoEnd(leds, colors.WHITE_STROBE).asProxy()));

    driverController
        .b()
        .onTrue(
            MechanismPositions.yeetCubeTeleop(
                    elevator,
                    stinger,
                    intake,
                    () -> yeetHeight.get(),
                    () -> yeetExtensionThreshold.get())
                .andThen(MechanismPositions.aggressiveZero(elevator, stinger)));

    // driverAssistController
    //     .povRight()
    //     .onTrue(Commands.runOnce(() -> RobotState.getInstance().incrementDesiredBay()));

    // driverAssistController
    //     .povLeft()
    //     .onTrue(Commands.runOnce(() -> RobotState.getInstance().decrementDesiredBay()));

    // driverAssistController
    //     .y()
    //     .onTrue(Commands.run(() -> vision.disableMaxDistanceAwayForTags(), vision))
    //     .onFalse(Commands.run(() -> vision.enableMaxDistanceAwayForTags(), vision));

    // driverController
    //     .start()
    //     .onTrue(Commands.runOnce(() -> drivetrain.resetModulesToAbsolute(), drivetrain));
  }

  /** configureAutoCommands - add autonomous routines to chooser */
  public void configureAutoCommands() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    autos = new SwerveAutos(drivetrain, intake, elevator, stinger);

    List<String> autoNames = autos.getAutonomousCommandNames();

    for (int i = 0; i < autoNames.size(); i++) {
      String name = autoNames.get(i);
      // System.out.println("configureAutoCommands: " + name);
      if (i == 0) {
        // Do nothing command must be first in list.
        autoChooser.addDefaultOption(name, autos.getChooserElement(name));
      } else {
        autoChooser.addOption(name, autos.getChooserElement(name));
      }
    }

    // TODO: add drive characterization command? maybe not necessary?
    // autoChooser.addOption(
    //   "Drive Characterization",
    //    new FeedForwardCharacterization(
    //        drivetrain,
    //        true,
    //        new FeedForwardCharacterizationData("drive"),
    //        drivetrain::runCharacterizationVolts,
    //        drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass name of the selected autonomous to the main {@link Robot}
   *
   * @return name of currently selected auton
   */
  public String getAutonomousCommandName() {
    return autoChooser.getSendableChooser().getSelected();
  }

  /**
   * Use this to pass the autonomous chooser Element to the main {@link Robot} class. The chooser
   * element contains the selected autonomous command, trajectory, and starting pose.
   *
   * @return the autonomous chooser element
   */
  public Supplier<AutoChooserElement> getSelectedAutonChooserElement() {
    Supplier<AutoChooserElement> chooserElement = autoChooser.get();
    if (chooserElement == null) {
      return null;
    }
    return chooserElement;
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public void stopAll() {
    elevator.stop();
    stinger.stop();
  }
}
