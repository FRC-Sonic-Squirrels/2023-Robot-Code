// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.driverassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
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

  // -------------RED---------------

  RED_COMMUNITY(BLUE_COMMUNITY),

  RED_HALF_COURT(BLUE_HALF_COURT),

  RED_SKIP_CHECKPOINT(BLUE_SKIP_CHECKPOINT),

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

      Translation2d redPoint =
          new Translation2d(bluePoint.getX(), FIELD_WIDTH_METERS - bluePoint.getY());

      newPoints[i] = redPoint;
    }

    this.points = newPoints;
  }

  public boolean insideBox(Translation2d pointToCheck) {

    // A point is in a polygon if a line from the point to infinity crosses the polygon an odd
    // number of times
    boolean odd = false;
    // int totalCrosses = 0; // this is just used for debugging
    // For each edge (In this case for each point of the polygon and the previous one)

    for (int i = 0, j = points.length - 1; i < points.length; i++) {
      // If a line from the point into infinity crosses this edge
      // One point needs to be above, one below our y coordinate

      var currentPoint = points[i];
      var prevPoint = points[j];

      if ((currentPoint.getY() > pointToCheck.getY()) != (prevPoint.getY() > pointToCheck.getY())) {
        // ...and the edge doesn't cross our Y corrdinate before our x coordinate (but between our x
        // coordinate and infinity)

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
