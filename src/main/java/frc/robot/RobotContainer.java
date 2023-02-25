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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.AutoChooserElement;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.SwerveAutos;
import frc.robot.commands.drive.DriveWithSetRotation;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.stinger.StingerManualControl;
import frc.robot.commands.stinger.StingerSetExtension;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal2023;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIO2023;
import frc.robot.subsystems.stinger.Stinger;
import frc.robot.subsystems.stinger.StingerIOReal;
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
            // new Vision(VisionConstants.LEFT_ROBOT_TO_CAMERA, new
            // VisionIOPhotonVision(CAMERA_NAME));
            // TODO: add intake when intake is done
            elevator = new Elevator(new ElevatorReal2023());
            stinger = new Stinger(new StingerIOReal());
            intake = new Intake(new IntakeIO2023());

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
            // new Vision(
            //     VisionConstants.LEFT_ROBOT_TO_CAMERA,
            //     new VisionIOSim(layout, drivetrain::getPose,
            // VisionConstants.LEFT_ROBOT_TO_CAMERA));

            // TODO: test to see if we can just add a second camera
            // new Vision(
            //     VisionConstants.RIGHT_ROBOT_TO_CAMERA,
            //     new VisionIOSim(
            //         layout, drivetrain::getPose, VisionConstants.RIGHT_ROBOT_TO_CAMERA));

            intake = new Intake(new IntakeIO() {});
            elevator = new Elevator(new ElevatorIO() {});

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
      // new Vision(VisionConstants.LEFT_ROBOT_TO_CAMERA, new VisionIO() {});
      new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
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

    stinger.setDefaultCommand(
        new StingerManualControl(stinger, elevator, () -> operatorController.getRightX()));

    elevator.setDefaultCommand(
        new ElevatorManualControl(elevator, () -> -operatorController.getLeftY()));

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

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    // Debug Swerve Commands
    // driverController.a().whileTrue(Commands.run(() -> drivetrain.drive(-1, 0, 0), drivetrain));
    // driverController.b().whileTrue(Commands.run(() -> drivetrain.drive(0, -1, 0), drivetrain));
    // driverController.x().whileTrue(Commands.run(() -> drivetrain.drive(0, 1, 0), drivetrain));
    // driverController.y().whileTrue(Commands.run(() -> drivetrain.drive(1, 0, 0), drivetrain));

    // FIXME: UNCOMMENT button bindings

    // toggle between Field and Robot centric driving
    driverController
        .b()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

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

    driverController
        .povDown()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    () -> driverController.getLeftY(),
                    () -> driverController.getLeftX(),
                    180)
                .until(() -> Math.abs(driverController.getRightX()) > 0.7));

    driverController
        .povUp()
        .onTrue(
            new DriveWithSetRotation(
                    drivetrain,
                    () -> driverController.getLeftY(),
                    () -> driverController.getLeftX(),
                    0)
                .until(() -> Math.abs(driverController.getRightX()) > 0.3));

    operatorController.y().whileTrue(new IntakeGrabCone(intake, 1.0));
    operatorController.x().whileTrue(new IntakeGrabCube(intake, 0.3));

    operatorController.b().whileTrue(new IntakeScoreCone(intake, 0.8));
    operatorController.a().whileTrue(new IntakeScoreCube(intake, 0.5));

    operatorController.rightBumper().onTrue(new ElevatorSetHeight(elevator, 42.0));
    operatorController.leftBumper().onTrue(new ElevatorSetHeight(elevator, 49.0));
    operatorController.rightTrigger(0.5).onTrue(new ElevatorSetHeight(elevator, 0.0));

    operatorController.povLeft().onTrue(new StingerSetExtension(stinger, 0));
    operatorController.povUp().onTrue(new StingerSetExtension(stinger, 10));
    operatorController.povRight().onTrue(new StingerSetExtension(stinger, 25));
  }

  /** configureAutoCommands - add autonomous routines to chooser */
  public void configureAutoCommands() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    autos = new SwerveAutos(drivetrain, intake);

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
