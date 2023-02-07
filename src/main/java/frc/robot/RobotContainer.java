// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.driverassist.GridPositionHandler;
import frc.lib.team2930.driverassist.GridPositionHandler.DeadzoneBox;
import frc.lib.team2930.driverassist.GridPositionHandler.EntranceCheckpoint;
import frc.lib.team2930.driverassist.GridPositionHandler.LogicalGridLocation;
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
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveAvoidBoxes;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);

  /* Driver Buttons */
  //   private final JoystickButton zeroGyro =
  //       new JoystickButton(driverController, XboxController.Button.kBack.value);
  //   private final JoystickButton robotCentric =
  //       new JoystickButton(driverController, XboxController.Button.kB.value);
  //   private final JoystickButton xStance =
  //       new JoystickButton(driverController, XboxController.Button.kA.value);
  //   private final JoystickButton intakeOut =
  //       new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

  private Drivetrain drivetrain;
  private Intake intake;

  private DriveToGridPosition autoDriveToGrid;
  public final GridPositionHandler gridPositionHandler = new GridPositionHandler();

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

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
            new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            intake = new Intake(new IntakeIOFalcon());
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
            // try {
            //   layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            // } catch (IOException e) {
            //   layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            // }
            // new Vision(
            //     new VisionIOSim(layout, drivetrain::getPose, VisionConstants.ROBOT_TO_CAMERA));
            new Vision(new VisionIO() {});

            new Pneumatics(new PneumaticsIO() {});
            intake = new Intake(new IntakeIO() {});
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
      new Vision(new VisionIO() {});
      new Pneumatics(new PneumaticsIO() {});
      intake = new Intake(new IntakeIO() {});
    }

    // workaround warning about unused variable
    // pneumatics.getPressure();

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

    // drivetrain.setDefaultCommand(
    //     new TeleopSwerve(
    //         drivetrain,
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         driverController::getRightX));

    drivetrain.setDefaultCommand(
        new DriveAvoidBoxes(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX));

    autoDriveToGrid = new DriveToGridPosition(drivetrain, intake, driverController);
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
    for (DeadzoneBox box : GridPositionHandler.allowAbleActivationAreaBlue) {
      box.Log();
    }

    for (DeadzoneBox box : GridPositionHandler.allowAbleActivationAreaRed) {
      box.Log();
    }

    GridPositionHandler.DeadzoneBox.TEST_DEADZONE.Log();

    var inside =
        GridPositionHandler.DeadzoneBox.RED_COMMUNITY.insideBox(
            drivetrain.getPose().getTranslation());

    Logger.getInstance().recordOutput("DriverAssist/GridPosition/insideBox", inside);

    EntranceCheckpoint.BLUE_WALL.log();
    EntranceCheckpoint.BLUE_HUMAN_PLAYER.log();

    EntranceCheckpoint.RED_WALL.log();
    EntranceCheckpoint.RED_HUMAN_PLAYER.log();

    for (LogicalGridLocation logicGrid : GridPositionHandler.logicalGridOrder) {
      logicGrid.bluePhysical.log();
      logicGrid.redPhysical.log();
    }
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle

    // robotCentric.toggleOnTrue(
    //     Commands.either(
    //         Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    //         Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
    //         drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    // zeroGyro.onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

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
                      var cmd =
                          autoDriveToGrid.testLogicalBay(
                              gridPositionHandler.getDesiredBay()); // some command

                      Command currentCmd = drivetrain.getCurrentCommand();

                      if (currentCmd instanceof OverrideDrivetrainStop) {
                        ((OverrideDrivetrainStop) currentCmd).overideStop();
                      }

                      // interupt command if joystick value is greater than 0.7 for 0.2 seconds
                      // cmd.until(anyJoystickInputAboveForTrigger(0.7, 0.2, driverController));
                      cmd.schedule();
                    })
                .beforeStarting(Commands.print("A")));

    driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> drivetrain.resetOdometry(new Pose2d(4.5, 1.13, new Rotation2d())),
                drivetrain));

    driverController
        .povRight()
        .onTrue(
            Commands.runOnce(() -> gridPositionHandler.incrementNextBay())
                .andThen(Commands.print("Incremented bay")));
    driverController
        .povLeft()
        .onTrue(Commands.runOnce(() -> gridPositionHandler.decrementNextBay()));

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

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    PathPlannerTrajectory testPath2mForward =
        PathPlanner.loadPath(
            "2mForward",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory testPath2mForward180 =
        PathPlanner.loadPath(
            "2mForward180",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory testPath3mForward360 =
        PathPlanner.loadPath(
            "3mForward360",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // PathPlannerTrajectory testAllianceFlipPath =
    //     PathPlanner.loadPath(
    //         "testPath",
    //         AUTO_MAX_SPEED_METERS_PER_SECOND,
    //         AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("2m Forward", new FollowPath(testPath2mForward, drivetrain, true));
    autoChooser.addOption(
        "2m Forward w/ 180", new FollowPath(testPath2mForward180, drivetrain, true));
    autoChooser.addOption(
        "3m Forward 2/ 360", new FollowPath(testPath3mForward360, drivetrain, true, true));
    // autoChooser.addOption(
    //     "test alliance flip path", new FollowPath(testAllianceFlipPath, drivetrain, true, true));
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));
    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
