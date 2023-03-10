// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Rotation2dUtils {

  public static Rotation2d flipCosine(Rotation2d toRot) {
    return new Rotation2d(-toRot.getCos(), toRot.getSin());
  }
}
