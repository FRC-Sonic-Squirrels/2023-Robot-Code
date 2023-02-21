package frc.robot.subsystems.drivetrain;

import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIOSim;

public final class DrivetrainConstantsSimbot extends DrivetrainConstants2022 {
  @Override
  public Drivetrain buildDriveTrain() {
    SwerveModule flModule =
        new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule frModule =
        new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule blModule =
        new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule brModule =
        new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);

    return new Drivetrain(this, new GyroIO() {}, flModule, frModule, blModule, brModule);
  }
}
