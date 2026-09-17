package org.firstinspires.ftc.teamcode.Math;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Simple class to represent matrix for transforming 2d vectors.
public class RotationMatrix2d {
    // Row major 2x3 matrix
    public double[] coeff = new double[4];

    public RotationMatrix2d() {
        // Identity matrix
        coeff[0] = 1;
        coeff[3] = 1;
    }

    public RotationMatrix2d(double angle, AngleUnit units) {
        // Create a rotationScalar matrix.
        setRotation(angle, units);
    }

    public RotationMatrix2d(double rotscale1, double rotscale2,
                            double rotscale3, double rotscale4) {
        coeff[0] = rotscale1;
        coeff[1] = rotscale2;
        coeff[2] = rotscale3;
        coeff[3] = rotscale4;
    }

    public void setRotation(double angle, AngleUnit units) {
        // turn this into a rotation matrix that rotates angle.
        if (units == AngleUnit.DEGREES) {
            angle = Math.toRadians(angle);
        }

        coeff[0] = Math.cos(angle);
        coeff[1] = -Math.sin(angle);
        coeff[2] = Math.sin(angle);
        coeff[3] = Math.cos(angle);
    }

    public Vector2 multiply(Vector2 vec) {
        return new Vector2(
                vec.x * coeff[0] + vec.y * coeff[1],
                vec.x * coeff[2] + vec.y * coeff[3]);
    }

    public void setIdentity()
    {
        coeff[0] = 1;
        coeff[1] = 0;
        coeff[2] = 0;
        coeff[3] = 1;
    }

    @NonNull
    @Override
    public String toString() {
        return coeff[0] + "," + coeff[1] + "," + coeff[2] + ";" + coeff[3];
    }
}
