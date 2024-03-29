// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public enum BoundingBoxes {
  BLUE_COMMUNITY(
      new Translation2d(1.4, 5.5),
      new Translation2d(3.4, 5.5),
      new Translation2d(3.4, 3.95),
      new Translation2d(2.95, 3.95),
      new Translation2d(2.95, 1.6),
      new Translation2d(4.9, 1.6),
      new Translation2d(4.9, 0),
      new Translation2d(1.4, 0)),

  BLUE_SKIP_CHECKPOINT(
      new Translation2d(1.4, 5.5),
      new Translation2d(2.9, 5.5),
      new Translation2d(2.9, 0.0),
      new Translation2d(1.4, 0.0)),

  BLUE_HALF_COURT(
      new Translation2d(3.35, 8.02),
      new Translation2d(8.3, 8.02),
      new Translation2d(8.3, 0.0),
      new Translation2d(4.85, 0.0),
      new Translation2d(4.85, 4.0),
      new Translation2d(3.35, 4.0)),

  BLUE_HP_STATION_ACTIVATION_AREA(
      // new Translation2d(5, 8.02),
      // new Translation2d(16.54, 8.02),
      // new Translation2d(16.54, 5.6),
      // new Translation2d(13, 5.6),
      // new Translation2d(13, 4.25),
      // new Translation2d(11.7, 4.25),
      // new Translation2d(11.7, 0),
      // new Translation2d(5, 0)

      new Translation2d(9.5, 8.25),
      new Translation2d(16.74, 8.25),
      new Translation2d(16.74, 5.3),
      new Translation2d(9.5, 5.3)),

  // -------------RED---------------

  RED_COMMUNITY(BLUE_COMMUNITY),

  RED_HALF_COURT(BLUE_HALF_COURT),

  RED_SKIP_CHECKPOINT(BLUE_SKIP_CHECKPOINT),

  RED_HP_STATION_ACTIVATION_AREA(BLUE_HP_STATION_ACTIVATION_AREA),

  TEST(
      new Translation2d(5, 5),
      new Translation2d(5, 6),
      new Translation2d(8, 7),
      new Translation2d(8, 8),
      new Translation2d(10, 5),
      new Translation2d(10, 0),
      new Translation2d(5, 0));

  private final Translation2d[] points;

  private static final double FIELD_WIDTH_METERS = GridPositionHandler.FIELD_WIDTH_METERS;

  private BoundingBoxes(Translation2d... points) {
    this.points = points;
  }

  private BoundingBoxes(BoundingBoxes blueToFlip) {
    Translation2d[] newPoints = new Translation2d[blueToFlip.points.length];
    for (int i = 0; i < newPoints.length; i++) {
      Translation2d bluePoint = blueToFlip.points[i];

      // Translation2d redPoint =
      //     new Translation2d(bluePoint.getX(), FIELD_WIDTH_METERS - bluePoint.getY());

      var redPoint =
          new Translation2d(
              Constants.FIELD_DIMENSIONS.FIELD_LENGTH_METERS - bluePoint.getX(), bluePoint.getY());

      newPoints[i] = redPoint;
    }

    this.points = newPoints;
  }

  public boolean insideBox(Translation2d pointToCheck) {
    /**
     * used this as a reference https://www.algorithms-and-technologies.com/point_in_polygon/java
     */

    // A point is in a polygon if a line from the point to infinity crosses the polygon an odd
    // number of times
    boolean odd = false;

    for (int i = 0, j = points.length - 1; i < points.length; i++) {

      var currentPoint = points[i];
      var prevPoint = points[j];

      // the pointToCheck has to be in between the Y of 2 points, doesn't work if they are both
      // below/above/the same
      if ((currentPoint.getY() > pointToCheck.getY()) != (prevPoint.getY() > pointToCheck.getY())) {

        // fancy math to figure out if the pointToCheck is between the 2 points
        if (pointToCheck.getX()
            < (prevPoint.getX() - currentPoint.getX())
                    * (pointToCheck.getY() - currentPoint.getY())
                    / (prevPoint.getY() - currentPoint.getY())
                + currentPoint.getX()) {

          odd = !odd;
        }
      }

      j = i;
    }

    return odd;
  }

  public void log(String ROOT) {
    List<State> states = new ArrayList<>();

    for (int i = 0; i < points.length; i++) {
      states.add(translationToState(points[i]));
    }

    states.add(translationToState(points[0]));

    Trajectory displayTrajectory = new Trajectory(states);

    Logger.getInstance()
        .recordOutput(ROOT + "/boundingBoxes/" + this.name() + "/traj/", displayTrajectory);
  }

  private State translationToState(Translation2d point) {
    return new State(0.0, 0.0, 0.0, new Pose2d(point, new Rotation2d()), 0.0);
  }
}
