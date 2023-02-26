package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class TeleopSwerve extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final Elevator elevator;
  private final Stinger stinger;

  private final TunableNumber elevatorUpTranslationMuliplier = new TunableNumber("teleopSwerve/elevatorUpTranslationMuliplier", 0.2);
  private final TunableNumber elevatorUpRotationalMultiplier = new TunableNumber("teleopSwerve/elevatorUpRotationalMultiplier", 0.1);

  private final TunableNumber stingerOutTranslationMuliplier = new TunableNumber("teleopSwerve/stingerOutTranslationMuliplier", 0.3);
  private final TunableNumber stingerOutRotationalMultiplier = new TunableNumber("teleopSwerve/stingerOutRotationalMultiplier", 0.4);


  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity
   */
  public TeleopSwerve(
      Drivetrain drivetrain,
      Elevator elevator, 
      Stinger stinger,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    this.elevator = elevator;
    this.stinger = stinger;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage = -modifyAxis(translationXSupplier.getAsDouble());
    double yPercentage = -modifyAxis(translationYSupplier.getAsDouble());
    double rotationPercentage = -modifyAxis(rotationSupplier.getAsDouble());

    double xMultiplier = 1.0; 
    double yMultiplier = 1.0;
    //0.6 driver starting preference
    double rotMultiplier = 0.6; 

    if(elevator.getHeightInches() > Constants.NODE_DISTANCES.STOW_HEIGHT + 5){
      xMultiplier -= elevatorUpTranslationMuliplier.get();
      yMultiplier -= elevatorUpTranslationMuliplier.get();

      rotMultiplier -= elevatorUpRotationalMultiplier.get();
    }

    if(this.stinger.getExtensionInches() > 8){
      xMultiplier -= stingerOutTranslationMuliplier.get();
      yMultiplier -= stingerOutTranslationMuliplier.get();

      rotMultiplier -= stingerOutRotationalMultiplier.get();
    }

    Logger.getInstance().recordOutput("TeleopSwerve/xMultiplier", xMultiplier);
    Logger.getInstance().recordOutput("TeleopSwerve/yMultiplier", yMultiplier);
    Logger.getInstance().recordOutput("TeleopSwerve/rotMultiplier", rotMultiplier);

    xPercentage *= xMultiplier;
    yPercentage *= yMultiplier;

    rotationPercentage *= rotMultiplier;

    double xVelocity = xPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double yVelocity = yPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double rotationalVelocity =
        rotationPercentage * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;


    // because our coordinate frame never changes from the bottom left, the "forward" direction in
    // code is always facing the red alliance wall
    // to make up on the joystick away from the red alliance wall we negate it when on the red
    // alliance
    if (DriverStation.getAlliance() == Alliance.Red) {
      xVelocity *= -1;
      yVelocity *= -1;
    }

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", true);
    Logger.getInstance().recordOutput("TeleopSwerve/xVelocity", xVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/yVelocity", yVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/rotationalVelocity", rotationalVelocity);

    drivetrain.drive(xVelocity, yVelocity, rotationalVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.stop();

    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DrivetrainConstants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
