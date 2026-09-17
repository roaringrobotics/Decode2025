package org.firstinspires.ftc.teamcode.Pathing;

public class PID {
    private double kP, kI, kD;
    private double previousError = 0;
    private double integral = 0;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        previousError = 0;
        integral = 0;
    }
}

