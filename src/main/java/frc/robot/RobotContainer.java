// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.AutoChooserElement;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.SwerveAutos;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.elevator.ElevatorFollowCurve;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.commands.stinger.StingerFollowCurve;
import frc.robot.commands.stinger.StingerManualControl;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainConstants2022;
import frc.robot.subsystems.drivetrain.DrivetrainConstants2023;
import frc.robot.subsystems.drivetrain.DrivetrainConstantsSimbot;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIO2022;
import frc.robot.subsystems.stinger.Stinger;
import frc.robot.subsystems.stinger.StingerIOReal;
import frc.robot.subsystems.stinger.StingerSim;
import frc.robot.subsystems.wrist.Wrist;
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

  private DrivetrainConstants drivetrainConstants;
  private Drivetrain drivetrain;
  private Intake intake;
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
            drivetrainConstants = new DrivetrainConstants2022();
            drivetrain = drivetrainConstants.buildDriveTrain();
            new Pneumatics(new PneumaticsIORev(false));

            intake = new Intake(new IntakeIO2022());

            // FIX ME i think the constants got killed in the merge
            // new Vision(
            //     new VisionIOPhotonVision(LEFT_CAMERA_NAME),
            //     new VisionIOPhotonVision(RIGHT_CAMERA_NAME));

            elevator = new Elevator(new ElevatorReal2022());

            break;
          }
        case ROBOT_2023_COMPBOT:
          {
            drivetrainConstants = new DrivetrainConstants2023();
            drivetrain = drivetrainConstants.buildDriveTrain();
            // TODO add vision subsystem
            // new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            // TODO: add intake when intake is done
            elevator = new Elevator(new ElevatorReal2023());
            stinger = new Stinger(new StingerIOReal());

            break;
          }

        case ROBOT_SIMBOT:
          {
            drivetrainConstants = new DrivetrainConstantsSimbot();
            drivetrain = drivetrainConstants.buildDriveTrain();
            AprilTagFieldLayout layout;
            try {
              layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }

            new Vision(
                drivetrain.getPoseEstimator(),
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.LEFT_ROBOT_TO_CAMERA),
                new VisionIOSim(
                    layout, drivetrain::getPose, VisionConstants.RIGHT_ROBOT_TO_CAMERA));

            new Pneumatics(new PneumaticsIO() {});
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
      drivetrainConstants = new DrivetrainConstantsSimbot();
      drivetrain = drivetrainConstants.buildDriveTrain();
      new Vision(drivetrain.getPoseEstimator(), new VisionIO() {}, new VisionIO() {});
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

    elevator.setDefaultCommand(
        new ElevatorManualControl(elevator, () -> -driverController.getRightY()));

    stinger.setDefaultCommand(
        new StingerManualControl(stinger, () -> driverController.getRightX()));

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

    driverController.a().onTrue(MechanismPositions.scoreConeHighPosition(elevator, stinger));
    driverController.b().onTrue(MechanismPositions.stowPosition(elevator, stinger));

    driverController
        .x()
        .whileTrue(new StingerFollowCurve(elevator, stinger).beforeStarting(Commands.print("X")));
    driverController
        .y()
        .whileTrue(new ElevatorFollowCurve(elevator, stinger).beforeStarting(Commands.print("X")));

    // driverController.a().onTrue((Commands.print("A")));

    // driverController.b().onTrue((Commands.print("B")));
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
}
