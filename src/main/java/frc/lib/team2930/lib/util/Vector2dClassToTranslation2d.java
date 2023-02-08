// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

/** This is a 2D vector struct that supports basic vector operations. */
public class Vector2dClassToTranslation2d {
    /**
     * Rotate a vector in Cartesian space.
     *
     * @param angle angle in degrees by which to rotate vector counter-clockwise.
     */
    public Translation2d rotate(Translation2d vec, double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = vec.getX() * cosA - vec.getY() * sinA;
        out[1] = vec.getX() * sinA + vec.getY() * cosA;
        return new Translation2d(out[0], out[1]);
    }

    /**
     * Returns dot product of this vector with argument.
     *
     * @param vec Vector with which to perform dot product.
     * @return Dot product of this vector with argument.
     */
    public static double dot(Translation2d vec1, Translation2d vec2) {
        return vec1.getX() * vec2.getX() + vec2.getY() * vec2.getY();
    }

    /**
     * Returns magnitude of vector.
     *
     * @return Magnitude of vector.
     */
    public static double magnitude(Translation2d vec) {
        return Math.sqrt(vec.getX() * vec.getX() + vec.getY() * vec.getY());
    }

    /**
     * Returns scalar projection of this vector onto argument.
     *
     * @param vec Vector onto which to project this vector.
     * @return scalar projection of this vector onto argument.
     */
    public double scalarProject(Translation2d vec1, Translation2d vec2) {
        return dot(vec1, vec2) / magnitude(vec1);
    }
}
