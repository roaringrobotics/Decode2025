package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class VelocityTestHardware {

    private final DcMotorEx shooterMotor;
    private final CRServo blueServo;
    private final CRServo blackServo;

    // Target velocity (ticks per second)
    private double targetVel = 0;
    private static final double P = 20;
    private static final double I = 0.3;
    private static final double D = 2.5;
    private static final double F = 10.2;

    // Velocity blocker settings
    private static final double allowedError = 50;  // TPS
    private static final long settleTimeMs = 90;    // ms
    private long stableSince = 0;

    public VelocityTestHardware(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooter");
        blueServo = hw.get(CRServo.class, "blueServo");
        blackServo = hw.get(CRServo.class, "blackServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Enable velocity control mode
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF for velocity control
        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F)
        );
    }

    public void setTargetVelocity(double vel) {
        targetVel = vel;
        shooterMotor.setVelocity(vel);
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public void stop() {
        setTargetVelocity(0);
    }


    public boolean isReady() {
        double current = shooterMotor.getVelocity();

        if (Math.abs(current - targetVel) < allowedError) {
            if (stableSince == 0)
                stableSince = System.currentTimeMillis();
            return System.currentTimeMillis() - stableSince > settleTimeMs;
        } else {
            stableSince = 0;
            return false;
        }
    }

    public void feed() {
        blueServo.setPower(-1);
        blackServo.setPower(1);
    }

    public void stopFeed() {
        blueServo.setPower(0);
        blackServo.setPower(0);
    }
}
