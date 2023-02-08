package frc.lib.team2930.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Utilities for manipulating Trajectories, Pose2d, Rotation2d, and Vector2d. */

// MIGHT NOT WORK, vector2d has been deprecated, I replaced it with translation2d
public class TrajectoryUtils {

    // private method that changes the rotation of a Pose2d (because that isn't already included in
    // the class for some reason)
    public static Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(pose.getX(), pose.getY(), rotation);
    }

    // private method that gets the angle between two Translation2ds
    public static Rotation2d getTranslationsAngle(Translation2d pose1, Translation2d pose2) {
        Translation2d vector1 = new Translation2d(pose1.getX(), pose1.getY());
        Translation2d vector2 = new Translation2d(pose2.getX(), pose2.getY());
        double dotProduct = Vector2dClassToTranslation2d.dot(vector1, vector2);
        double magnitude =
                Vector2dClassToTranslation2d.magnitude(vector1)
                        * Vector2dClassToTranslation2d.magnitude(vector2);

        return new Rotation2d(Math.acos(dotProduct / magnitude));
    }

    // private method that gets the angle between two Translation2ds
    public static double getTranslationsAngleDouble(Translation2d pose1, Translation2d pose2) {
        Translation2d vector1 = new Translation2d(pose1.getX(), pose1.getY());
        Translation2d vector2 = new Translation2d(pose2.getX(), pose2.getY());
        double dotProduct = Vector2dClassToTranslation2d.dot(vector1, vector2);
        double magnitude =
                Vector2dClassToTranslation2d.magnitude(vector1)
                        * Vector2dClassToTranslation2d.magnitude(vector2);

        return Math.acos(dotProduct / magnitude);
    }

    // private method that gets a midPos between two poses
    public static Translation2d midPosFinder(Translation2d start, Translation2d target) {
        // double hypotenuse = Math.hypot( (target.getX() - start.getX()), (target.getY() -
        // start.getY()) );
        double xDist = (target.getX() - start.getX()) * 0.75;
        double yDist = (target.getY() - start.getY()) * 0.75;
        Translation2d midPos = new Translation2d(start.getX() + xDist, start.getY() + yDist);

        return midPos;
    }

    public static Translation2d poseToTranslation(Pose2d pose) {
        return new Translation2d(pose.getX(), pose.getY());
    }
}
