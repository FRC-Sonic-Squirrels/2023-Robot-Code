package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

public class SwerveAutos {
  private Drivetrain drivetrain;
  private Intake intake;

  public SwerveAutos(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  public Command testPath2mForward() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward",
            AUTO_TEST_MAX_SPEED_METERS_PER_SECOND,
            AUTO_TEST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath2mForward180() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "2mForward180",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command testPath3mForward360() {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "3mForward360",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }

  public Command curve() {
    PathPlannerTrajectory path = 
      PathPlanner.loadPath(
        "curve", 
        AUTO_MAX_SPEED_METERS_PER_SECOND, 
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }
  public Command rotateAroundPoint() {
    PathPlannerTrajectory path = 
      PathPlanner.loadPath(
        "rotateAroundPoint", 
        AUTO_MAX_SPEED_METERS_PER_SECOND, 
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        return new SequentialCommandGroup(new FollowPath(path, drivetrain, true));
  }
}
