package org.firstinspires.ftc.teamcode.Pathing;

import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

public class PIDController {
    private double target;
    private double integralSum;
    private double lastError;
    private PidParams params;

    public PIDController(double kP, double kI, double kD) {
        params = new PidParams(kP, kI, kD);
        this.integralSum = 0;
        this.lastError = 0;
    }

    public PIDController(PidParams pidParams) {
        params = pidParams;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public void updateCoefficients(double kP, double kI, double kD)
    {
        params.kP = kP;
        params.kI = kI;
        params.kD = kD;
    }

    public void setTargetPoint(double target) {
        this.target = target;
    }

    public double calculate(double currentPoint, @NonNull TimeSourceI timeSource)
    {
        long deltaTimeMs = timeSource.deltaTimeMs();
        double deltaTimeSec = Constants.millisecondsToSeconds((double)deltaTimeMs);

        double error = target - currentPoint;
        integralSum += error * deltaTimeSec;

        double output;
        if(deltaTimeSec == 0) {
            // Avoid divide by zero it deltaTime is zero.
            double P = params.kP * error;
            double I = params.kI * integralSum;
            output = P + I;
        }
        else {
            double derivative = (error - lastError) / deltaTimeSec;
            double P = params.kP * error;
            double I = params.kI * integralSum;
            double D = params.kD * derivative;
            output = P + I + D;
        }

        lastError = error;

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }

    public void setCoeff(double kp, double ki, double kd) {
        params.kP = kp;
        params.kI = ki;
        params.kD = kd;
    }
}
