package org.firstinspires.ftc.teamcode;

public class Constants {
    static public final double mmToInch = 1.0/25.4;

    // encoderResolution is based on circumference of wheels and ticks per
    // revolution of Rev encoders.
    static public final double wheelDiameterMm = 32.0;
    static public final double wheelCircumference = 2.0*Math.PI*(wheelDiameterMm/2.0);
    static public final double revTicksPerRotation = 8192.0;
    static public final double encoderResolution = 19.894;

    // Force compile error so these are filled in.
    // Offsets of encoders from center of rotation in inches.
    // See page 3 of
    // https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
    
    // The arbitrary point in the diagram is where we want 0,0 field coordinate to be relative
    // to the robot.  The measurements can be from the grabber to simplify pathing.

    // For pinpoint y is strafe and x is forward.
    // Offset along the x-axis from where you want 0,0 to be relative to the robot
    // for the forward encoder.
    public static final double xForwardEncoderOffsetInches = 128*mmToInch;

    // Offset along the y-axis from where you want 0,0 to be relative to the robot
    // for the strafe encoder.
    public static final double yStrafeEncoderOffsetInches = 122*mmToInch;

    public static long millisecondsToSeconds(long ms)
    {
        return ms/1000;
    }

    public static double millisecondsToSeconds(double ms)
    {
        return ms/1000.0;
    }
}
