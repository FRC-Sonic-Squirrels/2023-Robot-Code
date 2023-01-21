package frc.robot.subsystems.wrist;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This subsystem represents the robot's arm system, which pnuematics to operate. */
public class Wrist extends SubsystemBase {

  /** Creates a new Arm. */
  Solenoid solenoid = null;

  public Wrist() {

    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
  }

  // TODO: figure out a way to use the constants WRIST_SOLENOID_DEPLOY and WRIST_SOLENOID_RETRACT
  public void solenoidUp() {
    solenoid.set(true);
  }

  public void solenoidDown() {
    solenoid.set(true);
  }

  public void stop() {
    solenoid.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
