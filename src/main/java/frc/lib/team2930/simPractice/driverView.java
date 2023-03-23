// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.simPractice;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team6328.util.TunableNumber;

/** Add your docs here. */
public class driverView {
  private final TunableNumber driverHeight = new TunableNumber("driverView/driverHeightFeet", 6);
  private double driverX;
  private double driverY;
  private double driverZ;
  private double driverRoll;
  private double driverPitch;
  private double driverYaw;
  private double[] driverYlocations = {1.28, 3.02, 4.91};

  private static driverView instance;

  public static driverView getInstance() {
    if (instance == null) {
      instance = new driverView();
    }
    return instance;
  }

  public Pose3d getDriverPose(Pose2d robotPose) {
    driverY = driverYlocations[DriverStation.getLocation() - 1];
    driverZ = Units.feetToMeters(driverHeight.get() - 0.5);
    if (DriverStation.getAlliance() == Alliance.Blue) {
      driverX = -0.57;
    } else {
      driverX = 17.11;
    }
    driverRoll = Math.toRadians(180);
    driverPitch =
        Math.atan2(
                -driverZ,
                Math.sqrt(
                    Math.pow(robotPose.getX() - driverX, 2)
                        + Math.pow(robotPose.getY() - driverY, 2)))
            + Math.toRadians(180);
    driverYaw = Math.atan((driverY - robotPose.getY()) / (driverX - robotPose.getX()));
    if (DriverStation.getAlliance() == Alliance.Blue) {
      driverYaw += Math.toRadians(180);
    }
    return new Pose3d(
        driverX, driverY, driverZ, new Rotation3d(driverRoll, driverPitch, driverYaw));
  }
}
