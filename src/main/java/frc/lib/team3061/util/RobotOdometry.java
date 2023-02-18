package frc.lib.team3061.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private final SwerveDrivePoseEstimator estimator;
  private final SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public RobotOdometry(DrivetrainConstants constants) {
    estimator =
        new SwerveDrivePoseEstimator(
            constants.KINEMATICS, new Rotation2d(), defaultPositions, new Pose2d());
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return estimator;
  }
}
