package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor topMotor;
    private final DcMotor bottomMotor;

    public Shooter(HardwareMap hardwareMap) {
        topMotor = hardwareMap.get(DcMotor.class, "shooterTop");
        bottomMotor = hardwareMap.get(DcMotor.class, "shooterBottom");

        // Reverse bottom motor so both spin the same physical direction
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default to BRAKE when power is zero
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Default run mode (change as needed)
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTopPower(double power) {
        topMotor.setPower(power);
    }

    public void setBottomPower(double power) {
        bottomMotor.setPower(power);
    }

    public void setBothPower(double power) {
        setTopPower(power);
        setBottomPower(power);
    }

    public void setPowers(double topPower, double bottomPower) {
        setTopPower(topPower);
        setBottomPower(bottomPower);
    }

    public double getTopPower() {
        return topMotor.getPower();
    }

    public double getBottomPower() {
        return bottomMotor.getPower();
    }

    public void stop() {
        setBothPower(0.0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        topMotor.setZeroPowerBehavior(behavior);
        bottomMotor.setZeroPowerBehavior(behavior);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        topMotor.setMode(mode);
        bottomMotor.setMode(mode);
    }
}