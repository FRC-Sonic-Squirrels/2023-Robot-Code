// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * AutoChooserElement
 *
 * <p>This class holds a single SequentialCommandGroup and the associated trajectory and starting
 * pose, if any. The AutoChooserElement is designed to create a linked list so that commands can be
 * chained together.
 */
public class AutoChooserElement {
  private Pose2d initialPose;
  private Trajectory trajectory;
  private SequentialCommandGroup command;
  private AutoChooserElement next;

  public AutoChooserElement(PathPlannerTrajectory trajectory, SequentialCommandGroup command) {

    // Reduce the number of states in the trajectory. The trajectory is just for displaying
    // on the dashboard and not following. Reducing the trajectory size makes computation and
    // transmitting the data on NetworkTables faster.
    this.trajectory = decimateTrajectory(trajectory, 10);
    this.initialPose = null;
    if (trajectory != null) {
      this.initialPose = trajectory.getInitialHolonomicPose();
    }
    this.command = command;
    this.next = null;
  }

  public String toString() {
    String s = "pose: ";
    if (initialPose != null) {
      s += initialPose.getTranslation().toString();
    } else {
      s += "NONE";
    }
    s += "\n";
    s += "trajectory: ";
    if (trajectory != null) {
      // s += trajectory.toString();
      // s += "time: " + trajectory.getTotalTimeSeconds();
      s += "tStartPose:" + trajectory.getInitialPose().getTranslation();
      s +=
          "tEndPose:"
              + trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation();
    } else {
      s += "NONE";
    }
    s += "\n";
    if (next != null) {
      s += "Next:\n";
      s += next.toString();
    } else {
      s += " END";
    }

    return s;
  }

  /**
   * setNext - append a new AutoChooserElement to a list of AutoChooserElements
   *
   * @param append the AutoChooserElement to append to the list
   * @return AutoChooserElement
   */
  public AutoChooserElement setNext(AutoChooserElement append) {
    if (this.next == null) {
      this.next = append;
    } else {
      next.setNext(append);
    }
    return this;
  }

  /**
   * setNext - append a new path follow command using PathPlanner event markers to trigger commands.
   *
   * @param path
   * @param initialPath set true if this is the first command the autonomous command will follow
   * @param drivetrain drivetrain subsystem
   * @param eventMap a HashMap of events to be triggered by event markers
   * @return AutoChooserElement list with new command appended to the end
   */
  public AutoChooserElement setNext(
      PathPlannerTrajectory path,
      boolean initialPath,
      Drivetrain drivetrain,
      HashMap<String, Command> eventMap) {

    return this.setNext(
        new AutoChooserElement(
            path,
            new SequentialCommandGroup(
                new FollowPathWithEvents(
                    new FollowPath(path, drivetrain, initialPath), path.getMarkers(), eventMap))));
  }

  /**
   * setNext - Add a command with no path follow.
   *
   * @param command
   * @return AutoChooserElement
   */
  public AutoChooserElement setNext(SequentialCommandGroup command) {
    return this.setNext(new AutoChooserElement(null, command));
  }

  public Command getCommand() {
    if (next != null) {
      return new SequentialCommandGroup(command, next.getCommand());
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
    Trajectory nextTrajectory = null;

    if (next != null) {
      nextTrajectory = next.getTrajectory();
    }

    if ((fullTrajectory != null) && (nextTrajectory != null)) {
      fullTrajectory = fullTrajectory.concatenate(nextTrajectory);
    } else if (trajectory == null) {
      fullTrajectory = nextTrajectory;
    }

    return fullTrajectory;
  }

  /**
   * Decimates a trajectory by only including every nth element, specified by the modulus. The first
   * and last elements are always included to preserve the start and endpoints.
   *
   * @param detailed The trajectory to decimate.
   * @param modulus The modulus of the decimate command
   * @return The concatenated trajectory.
   */
  public static Trajectory decimateTrajectory(Trajectory detailed, int modulus) {

    if (detailed == null) {
      return detailed;
    }

    List<State> states = new ArrayList<State>();

    for (int i = 0; i < detailed.getStates().size(); ++i) {
      var s = detailed.getStates().get(i);

      if (i == 0 || i == detailed.getStates().size() || (i % modulus == 0)) {
        states.add(
            new State(
                s.timeSeconds,
                s.velocityMetersPerSecond,
                s.accelerationMetersPerSecondSq,
                s.poseMeters,
                s.curvatureRadPerMeter));
      }
    }

    return new Trajectory(states);
  }
}
