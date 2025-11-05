package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;

    DriveTrain (HardwareMap hardwareMap)
    {

        // Drive Motors
        // Names need to match configuration on driver hub
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

//      For this Robot(Pickle), you have to make the right wheels reversed.

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setFrontLeftPower(double power) {
        frontLeft.setPower(power);
    }

    public void setFrontRightPower(double power) {
        frontRight.setPower(power);

    }

    public void setBackLeftPower(double power) {
        backLeft.setPower(power);

    }

    public void setBackRightPower(double power) {
        backRight.setPower(power);
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }

    public void stopMotors() {
        setFrontLeftPower(0);
        setFrontRightPower(0);
        setBackLeftPower(0);
        setBackRightPower(0);
    }


}
