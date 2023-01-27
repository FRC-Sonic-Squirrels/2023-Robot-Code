// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoChooserElement {
  private Pose2d initialPose;
  private Trajectory trajectory;
  private Supplier<SequentialCommandGroup> commandSupplier;
  private AutoChooserElement next;

  AutoChooserElement(
      PathPlannerTrajectory trajectory, Supplier<SequentialCommandGroup> commandSupplier) {
    this.trajectory = trajectory;
    this.initialPose = null;
    if (trajectory != null) {
      this.initialPose = trajectory.getInitialHolonomicPose();
    }
    this.commandSupplier = commandSupplier;
    this.next = null;
  }

  public AutoChooserElement setNext(AutoChooserElement next) {
    if (this.next == null) {
      this.next = next;
    } else {
      DriverStation.reportError(
          "AutoChooserElement next already populated", Thread.currentThread().getStackTrace());
    }
    return this;
  }

  public Command getCommand() {
    SequentialCommandGroup command = commandSupplier.get();
    if (next != null) {
      command.addCommands(next.getCommand());
    }
    return command;
  }

  public Pose2d getPose2d() {
    if ((initialPose == null) && (next != null)) {
      initialPose = next.getPose2d();
    }
    if (initialPose == null) {
      return new Pose2d();
    }
    return initialPose;
  }

  public Trajectory getTrajectory() {
    Trajectory fullTrajectory = trajectory;
    if ((trajectory == null) && (next != null)) {
      fullTrajectory = next.getTrajectory();
    } else if ((trajectory != null) && (next != null)) {
      fullTrajectory.concatenate(next.getTrajectory());
    }
    if (fullTrajectory == null) {
      return new Trajectory();
    }

    return fullTrajectory;
  }
}
