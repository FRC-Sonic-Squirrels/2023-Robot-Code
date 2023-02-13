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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AutoChooserElement;
import frc.lib.team2930.driverassist.DeadzoneBox;
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.HumanLoadingStationHandler.LoadingStationLocation;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.SwerveAutos;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal2022;
import frc.robot.subsystems.elevator.ElevatorReal2023;
import frc.robot.subsystems.elevator.ElevatorSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIO2022;
import frc.robot.subsystems.stinger.Stinger;
import frc.robot.subsystems.stinger.StingerIOReal;
import frc.robot.subsystems.stinger.StingerSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSolenoid;
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

  private Drivetrain drivetrain;
  private Intake intake;

  private DriveToGridPosition autoDriveToGrid;
  public final GridPositionHandler gridPositionHandler = GridPositionHandler.getInstance();
  public SwerveAutos autos;
  private Stinger stinger;
  private Elevator elevator;
  private Wrist wrist;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private LoggedDashboardChooser<Supplier<AutoChooserElement>> autoChooser;

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

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            new Pneumatics(new PneumaticsIORev(false));

            intake = new Intake(new IntakeIO2022());

            // FIX ME i think the constants got killed in the merge
            // new Vision(
            //     new VisionIOPhotonVision(LEFT_CAMERA_NAME),
            //     new VisionIOPhotonVision(RIGHT_CAMERA_NAME));

            elevator = new Elevator(new ElevatorReal2022());

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
            new Pneumatics(new PneumaticsIORev(false));
            // TODO add vision subsystem
            // new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            // TODO: add intake when intake is done
            elevator = new Elevator(new ElevatorReal2023());
            stinger = new Stinger(new StingerIOReal());

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
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }

            new Vision(
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.LEFT_ROBOT_TO_CAMERA),
                new VisionIOSim(
                    layout, drivetrain::getPose, VisionConstants.RIGHT_ROBOT_TO_CAMERA));

            new Pneumatics(new PneumaticsIO() {});
            intake = new Intake(new IntakeIO() {});

            elevator = new Elevator(new ElevatorSim());
            stinger = new Stinger(new StingerSim());

            wrist = new Wrist(new WristIO() {});

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
      new Vision(new VisionIO() {}, new VisionIO() {});
      new Elevator(new ElevatorIO() {});
      new Pneumatics(new PneumaticsIO() {});
      intake = new Intake(new IntakeIO() {});
      wrist = new Wrist(new WristIOSolenoid() {});
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

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX));

    // drivetrain.setDefaultCommand(
    //     new DriveAvoidBoxes(
    //         drivetrain,
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         driverController::getRightX));

    autoDriveToGrid = new DriveToGridPosition(drivetrain, intake, driverController);

    // elevator.setDefaultCommand(
    //     new ElevatorManualControl(elevator, () -> -driverController.getRightY()));

    // stinger.setDefaultCommand(
    //     new StingerManualControl(stinger, () -> driverController.getRightX()));

    // elevator.setDefaultCommand(
    //     new ElevatorControlCommand(
    //         elevator, operatorController, Constants.ElevatorConstants.elevatorSpeedMultiplier));

    configureButtonBindings();
    configureAutoCommands();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  public void testBox() {
    // DeadzoneBox.BLUE_SKIP_CHECKPOINT.Log();

    for (DeadzoneBox box : GridPositionHandler.allowAbleActivationAreaBlue) {
      box.Log();
    }

    for (DeadzoneBox box : GridPositionHandler.allowAbleActivationAreaRed) {
      box.Log();
    }

    // GridPositionHandler.DeadzoneBox.TEST_DEADZONE.Log();

    // var inside =
    //     GridPositionHandler.DeadzoneBox.RED_COMMUNITY.insideBox(
    //         drivetrain.getPose().getTranslation());

    // Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox", inside);

    // EntranceCheckpoint.BLUE_WALL.log();
    // EntranceCheckpoint.BLUE_HUMAN_PLAYER.log();

    // EntranceCheckpoint.RED_WALL.log();
    // EntranceCheckpoint.RED_HUMAN_PLAYER.log();

    for (LogicalGridLocation logicGrid : GridPositionHandler.logicalGridOrder) {
      logicGrid.bluePhysical.log();
      logicGrid.redPhysical.log();
    }
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle

    // driverController
    //     .b()
    //     .toggleOnTrue(
    //         Commands.either(
    //             Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    //             Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
    //             drivetrain::getFieldRelative));

    // // reset gyro to 0 degrees
    // driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // // x-stance
    // driverController.a().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    // driverController.a().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // // intake
    // driverController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.runOnce(intake::extend, intake)
    //             .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.5), intake)));
    // driverController
    //     .rightBumper()
    //     .onFalse(
    //         Commands.runOnce(intake::retract, intake)
    //             .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.0), intake)));

    // driverController
    //     .povDown()
    //     .onTrue(
    //         new DriveWithSetRotation(
    //                 drivetrain,
    //                 () -> driverController.getLeftY(),
    //                 () -> driverController.getLeftX(),
    //                 180)
    //             .until(() -> Math.abs(driverController.getRightX()) > 0.7));

    // driverController
    //     .povUp()
    //     .onTrue(
    //         new DriveWithSetRotation(
    //                 drivetrain,
    //                 () -> driverController.getLeftY(),
    //                 () -> driverController.getLeftX(),
    //                 0)
    //             .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    // driverController
    //     .a()
    //     .onTrue(new ElevatorSetHeight(elevator, 20).beforeStarting(Commands.print("A")));

    // driverController
    //     .b()
    //     .onTrue(new ElevatorSetHeight(elevator, 0.0).beforeStarting(Commands.print("B")));

    // driverController
    //     .x()
    //     .onTrue(new StingerSetExtension(stinger, 0).beforeStarting(Commands.print("X")));

    // driverController
    //     .y()
    //     .onTrue(new StingerSetExtension(stinger, 25).beforeStarting(Commands.print("Y")));

    // driverController.a().onTrue(MechanismPositions.scoreConeHighPosition(elevator, stinger));
    // driverController.b().onTrue(MechanismPositions.stowPosition(elevator, stinger));

    // driverController
    //     .x()
    //     .whileTrue(new StingerFollowCurve(elevator,
    // stinger).beforeStarting(Commands.print("X")));
    // driverController
    //     .y()
    //     .whileTrue(new ElevatorFollowCurve(elevator,
    // stinger).beforeStarting(Commands.print("X")));

    // x-stance
    // xStance.onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    // xStance.onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // // intake
    // intakeOut.whileTrue(
    //     Commands.runOnce(intake::extend, intake)
    //         .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.5), intake)));
    // intakeOut.onFalse(
    //     Commands.runOnce(intake::retract, intake)
    //         .andThen(Commands.runOnce(() -> intake.runIntakePercent(0.0), intake)));

    driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // driverController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               var cmd =
    //                   autoDriveToGrid.driveToGridPosAndScore(GridPositions.GRID_8); // some
    // command
    //               // you can do this because Trigger implements BooleanSupplier
    //               cmd.schedule();
    //             }));

    driverController
        .a()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var scoringSequence =
                          MechanismPositions.scoreConeHighPosition(elevator, stinger)
                              .andThen(
                                  Commands.waitSeconds(0.4)
                                      .andThen(MechanismPositions.stowPosition(elevator, stinger)));

                      var cmd =
                          autoDriveToGrid.testLogicalBay(
                              GridPositionHandler.getDesiredBay(), scoringSequence); // some command

                      Command currentCmd = drivetrain.getCurrentCommand();

                      if (currentCmd instanceof OverrideDrivetrainStop) {
                        ((OverrideDrivetrainStop) currentCmd).overideStop();
                      }

                      // interupt command if joystick value is greater than 0.7 for 0.2 seconds
                      // cmd.until(anyJoystickInputAboveForTrigger(0.7, 0.2, driverController));
                      // var scoreCmd = MechanismPositions.scoreConeHighPosition(elevator, stinger);
                      // var retractCmd = MechanismPositions.stowPosition(elevator, stinger);

                      // cmd =
                      //     cmd.andThen(scoreCmd)
                      //         .andThen(Commands.waitSeconds(0.5).andThen(retractCmd));

                      cmd.schedule();
                    })
                .beforeStarting(Commands.print("A")));

    driverController.leftTrigger().onTrue(MechanismPositions.stowPosition(elevator, stinger));

    // driverController.a().onTrue(autoDriveToGrid.driveToGridPoseCommand());

    // driverController
    //     .leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               var cmd = autoDriveToGrid.humanPlayerStation(LoadingStationLocation.LEFT);

    //               Command currentCmd = drivetrain.getCurrentCommand();

    //               if (currentCmd instanceof OverrideDrivetrainStop) {
    //                 ((OverrideDrivetrainStop) currentCmd).overideStop();
    //               }

    //               // interupt command if joystick value is greater than 0.7 for 0.2 seconds
    //               // cmd.until(anyJoystickInputAboveForTrigger(0.7, 0.2, driverController));
    //               cmd.schedule();
    //             }));

    driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // make intake spin
                  var pickUpSequence =
                      new IntakeGrabCone(intake)
                          .until(() -> true)
                          .andThen(MechanismPositions.substationPickupPosition(elevator));
                  // stop intake spinning
                  var retractSequence =
                      Commands.runOnce(() -> intake.stop(), intake)
                          .andThen(MechanismPositions.stowPosition(elevator, stinger));

                  var cmd =
                      autoDriveToGrid.humanPlayerStation(
                          LoadingStationLocation.RIGHT, pickUpSequence, retractSequence);

                  Command currentCmd = drivetrain.getCurrentCommand();

                  if (currentCmd instanceof OverrideDrivetrainStop) {
                    ((OverrideDrivetrainStop) currentCmd).overideStop();
                  }

                  // interupt command if joystick value is greater than 0.7 for 0.2 seconds
                  // cmd.until(anyJoystickInputAboveForTrigger(0.7, 0.2, driverController));
                  cmd.schedule();
                }));

    driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> drivetrain.resetOdometry(new Pose2d(4.5, 1.13, new Rotation2d())),
                drivetrain));

    driverController
        .povRight()
        .onTrue(
            Commands.runOnce(() -> GridPositionHandler.incrementNextBay())
                .andThen(Commands.print("Incremented bay")));
    driverController
        .povLeft()
        .onTrue(Commands.runOnce(() -> GridPositionHandler.decrementNextBay()));

    driverController
        .rightTrigger()
        .whileTrue(
            new TeleopSwerve(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX));

    // PathPlannerTrajectory testAllianceFlipPath =
    //     PathPlanner.loadPath(
    //         "testPath",
    //         AUTO_MAX_SPEED_METERS_PER_SECOND,
    //         AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // driverController.y().onTrue(new FollowPath(testAllianceFlipPath, drivetrain, true, true));

    // .until(anyJoystickInputAboveForTrigger(0.5, 0.2, driverController)));

    // driverController
    //     .a()
    //     .onTrue(
    //         autoDriveToGrid
    //             .driveToCommunityCheckPointBasedOnPos()
    //             .until(anyJoystickInputAboveForTrigger(0.5, 0.2, driverController)));
  }

  /** configureAutoCommands - add autonomous routines to chooser */
  public void configureAutoCommands() {

    autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    autos = new SwerveAutos(drivetrain, intake);

    List<String> autoNames = autos.getAutonomousCommandNames();

    for (int i = 0; i < autoNames.size(); i++) {
      String name = autoNames.get(i);
      System.out.println("configureAutoCommands: " + name);
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

  public Trigger anyJoystickInputAboveForTrigger(
      double threshold, double forSeconds, CommandXboxController controller) {
    return new Trigger(
            () ->
                (controller.getLeftX() > threshold
                    || controller.getLeftY() > threshold
                    || controller.getRightX() > threshold
                    || controller.getRightY() > threshold))
        .debounce(forSeconds);
  }
}
