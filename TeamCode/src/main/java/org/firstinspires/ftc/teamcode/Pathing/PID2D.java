package org.firstinspires.ftc.teamcode.Pathing;

/**
 * 2D Position-based PID controller.
 * Calculates error in X and Y independently and applies PID control to each axis.
 * Can be used for autonomous movement to target positions.
 */
public class PID2D {
    private double kP, kI, kD;
    private double lastErrorX = 0, lastErrorY = 0;
    private double integralX = 0, integralY = 0;
    private long lastTime = System.nanoTime();

    public PID2D(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastTime = System.nanoTime();
    }

    /**
     * Calculate PID output for 2D position control.
     * 
     * @param targetX target X position
     * @param targetY target Y position
     * @param currentX current X position
     * @param currentY current Y position
     * @return double[] {outputX, outputY} - the PID output for each axis
     */
    public double[] calculate(double targetX, double targetY, double currentX, double currentY) {
        long now = System.nanoTime();
        double dt = Math.max(1e-6, (now - lastTime) / 1e9);
        lastTime = now;

        // Calculate error for each axis
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        // Update integral terms (with anti-windup clamp)
        integralX += errorX * dt;
        integralY += errorY * dt;
        integralX = clamp(integralX, -1.0, 1.0);
        integralY = clamp(integralY, -1.0, 1.0);

        // Calculate derivative terms
        double derivativeX = (errorX - lastErrorX) / dt;
        double derivativeY = (errorY - lastErrorY) / dt;

        
        lastErrorX = errorX;
        lastErrorY = errorY;

        // Calculate PID outputs for each axis
        double outputX = kP * errorX + kI * integralX + kD * derivativeX;
        double outputY = kP * errorY + kI * integralY + kD * derivativeY;

        return new double[]{outputX, outputY};
    }

    /**
     * Calculate PID output and clamp to max magnitude.
     * Useful when you have a maximum power constraint.
     * 
     * @param targetX target X position
     * @param targetY target Y position
     * @param currentX current X position
     * @param currentY current Y position
     * @param maxMagnitude maximum magnitude of output vector (e.g., 1.0 for full power)
     * @return double[] {outputX, outputY} - the clamped PID output for each axis
     */
    public double[] calculateClamped(double targetX, double targetY, double currentX, double currentY, double maxMagnitude) {
        double[] output = calculate(targetX, targetY, currentX, currentY);
        
        // Calculate magnitude of output vector
        double magnitude = Math.sqrt(output[0] * output[0] + output[1] * output[1]);
        
        if (magnitude > maxMagnitude && magnitude > 1e-6) {
            output[0] = output[0] / magnitude * maxMagnitude;
            output[1] = output[1] / magnitude * maxMagnitude;
        }
        
        return output;
    }

    /**
     * Reset integral and derivative history.
     * Call this when starting a new movement command.
     */
    public void reset() {
        lastErrorX = 0;
        lastErrorY = 0;
        integralX = 0;
        integralY = 0;
        lastTime = System.nanoTime();
    }

    /**
     * Set new PID gains.
     */
    public void setPIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Get current error magnitude (distance to target).
     * 
     * @param targetX target X position
     * @param targetY target Y position
     * @param currentX current X position
     * @param currentY current Y position
     * @return Euclidean distance to target
     */
    public static double getErrorMagnitude(double targetX, double targetY, double currentX, double currentY) {
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
