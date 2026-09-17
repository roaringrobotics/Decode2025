package Fakes;


/**
 * A class to simulate a mecanum drive robot, computing wheel speeds from desired
 * velocities and updating the robot's pose (x, y, theta) over time.
 This was create by xAI Grok as a possible upgrade to the mecanum drive sim in FakeHardware.
*/

public class MecanumDriveSim {
    // Robot chassis dimensions (in inches)
    private final double trackWidth;  // Distance between left and right wheels
    private final double trackLength; // Distance between front and back wheels

    // Current robot pose (x, y in inches; theta in radians)
    private double x;      // X-coordinate (forward/backward)
    private double y;      // Y-coordinate (left/right)
    private double theta;  // Orientation (angle in radians)

    // Wheel speeds (in inches per second)
    private double frontLeftSpeed;
    private double frontRightSpeed;
    private double backLeftSpeed;
    private double backRightSpeed;

    /**
     * Constructor for MecanumDrive.
     *
     * @param trackWidth   Distance between left and right wheels (inches)
     * @param trackLength  Distance between front and back wheels (inches)
     * @param initialX     Initial x-coordinate (inches)
     * @param initialY     Initial y-coordinate (inches)
     * @param initialTheta Initial orientation (radians)
     * @throws IllegalArgumentException if trackWidth or trackLength is non-positive
     */
    public MecanumDriveSim(double trackWidth, double trackLength,
                        double initialX, double initialY, double initialTheta) {
        if (trackWidth <= 0 || trackLength <= 0) {
            throw new IllegalArgumentException("Track width and length must be positive");
        }
        this.trackWidth = trackWidth;
        this.trackLength = trackLength;
        this.x = initialX;
        this.y = initialY;
        this.theta = initialTheta;
        this.frontLeftSpeed = 0.0;
        this.frontRightSpeed = 0.0;
        this.backLeftSpeed = 0.0;
        this.backRightSpeed = 0.0;
    }

    /**
     * Computes wheel speeds for desired robot velocities (inverse kinematics).
     *
     * @param vx  Desired linear velocity in x-direction (inches/second, forward/backward)
     * @param vy  Desired linear velocity in y-direction (inches/second, left/right)
     * @param omega Desired angular velocity (radians/second)
     */
    public void setVelocities(double vx, double vy, double omega) {
        // Mecanum drive inverse kinematics
        // Wheel speeds = linear velocities + angular velocity contribution
        double r = (trackWidth + trackLength) / 2.0; // Effective radius for rotation
        frontLeftSpeed = vx + vy + omega * r;
        frontRightSpeed = vx - vy - omega * r;
        backLeftSpeed = vx - vy + omega * r;
        backRightSpeed = vx + vy - omega * r;
    }

    /**
     * Simulates the robot's motion over a time step (forward kinematics).
     * Updates the robot's pose (x, y, theta) based on current wheel speeds.
     *
     * @param dt Time step (seconds)
     * @throws IllegalArgumentException if dt is negative
     */
    public void updatePose(double dt) {
        if (dt < 0) {
            throw new IllegalArgumentException("Time step cannot be negative");
        }

        // Forward kinematics: Compute robot velocities from wheel speeds
        double r = (trackWidth + trackLength) / 2.0;
        // Average velocities in robot frame
        double vxRobot = (frontLeftSpeed + frontRightSpeed + backLeftSpeed + backRightSpeed) / 4.0;
        double vyRobot = (frontLeftSpeed - frontRightSpeed - backLeftSpeed + backRightSpeed) / 4.0;
        double omega = (frontLeftSpeed - frontRightSpeed + backLeftSpeed - backRightSpeed) / (4.0 * r);

        // Transform velocities to global frame
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        double vxGlobal = vxRobot * cosTheta - vyRobot * sinTheta;
        double vyGlobal = vxRobot * sinTheta + vyRobot * cosTheta;

        // Update pose using Euler integration
        x += vxGlobal * dt;
        y += vyGlobal * dt;
        theta += omega * dt;

        // Normalize theta to [-pi, pi] for consistency
        theta = normalizeAngle(theta);
    }

    /**
     * Normalizes an angle to the range [-pi, pi].
     *
     * @param angle Angle in radians
     * @return Normalized angle in radians
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return Array of [x, y, theta] (inches, inches, radians)
     */
    public double[] getPose() {
        return new double[]{x, y, theta};
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return Array of [frontLeft, frontRight, backLeft, backRight] speeds (inches/second)
     */
    public double[] getWheelSpeeds() {
        return new double[]{frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed};
    }

    /**
     * Gets the track width of the robot.
     *
     * @return Track width in inches
     */
    public double getTrackWidth() {
        return trackWidth;
    }

    /**
     * Gets the track length of the robot.
     *
     * @return Track length in inches
     */
    public double getTrackLength() {
        return trackLength;
    }
}