package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;
    public FieldCentricPowerLevels fcPowerLevels = new FieldCentricPowerLevels();

    public DriveTrain(HardwareMap hardwareMap)
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
    private static class FieldCentricPowerLevels {
        public double frontLeftPower;
        public double backLeftPower;
        public double frontRightPower;
        public double backRightPower;

        public FieldCentricPowerLevels() {

        }
    }
    public void driveFieldCentric(double leftStickY, double leftStickX, double rightStickX,
                                   double powerScale, Hardware hw) {
        // Takes joystick input and sets motor power levels to drive
        // robot field centric.
        hw.imuPos.update();
        double botHeading = hw.imuPos.getHeading(AngleUnit.RADIANS);

        getFieldCentricPowerLevels(
                leftStickY, leftStickX,
                rightStickX, botHeading);

        frontLeft.setPower(fcPowerLevels.frontLeftPower * powerScale);
        backLeft.setPower(fcPowerLevels.backLeftPower * powerScale);
        frontRight.setPower(fcPowerLevels.frontRightPower * powerScale);
        backRight.setPower(fcPowerLevels.backRightPower * powerScale);
    }
    private void getFieldCentricPowerLevels(
            double leftStickY, double leftStickX,
            double rightStickX, double botHeading) {

        // leftStickY forward/backward [-1.0, 1.0]
        // leftStickX strafe [-1.0, 1.0]
        // rightStickX rotate [-1.0, 1.0].

        // Rotate joystick input vectors for field centric control.

        // Apply rotation matrix inverse of botHeading (-botHeading).
        // | cos(-h) -sin(-h) | | lsx  lsy |
        // | sin(-h)  cos(-h) | | lsx  lsy |
        // h = botHeading
        // lsx = left stick x value
        // lsy = left stick y value.
        double rotatedX = leftStickX * Math.cos(-botHeading) - leftStickY * Math.sin(-botHeading);
        double rotatedY = leftStickX * Math.sin(-botHeading) + leftStickY * Math.cos(-botHeading);

        // Don't know why the rotated value of left stick x is scale by 1.1.
        // It's not in online example code.
        rotatedX = rotatedX * 1.1;

        // Normalize output power [-1.0-1.0]
        double vectorSum = Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rightStickX);
        double normalize = 1.0 / Math.max(vectorSum, 1.0);

        // Set normalized field centric power levels.
        fcPowerLevels.frontLeftPower = (rotatedY + rotatedX + rightStickX) * normalize;
        fcPowerLevels.backLeftPower = (rotatedY - rotatedX + rightStickX) * normalize;
        fcPowerLevels.frontRightPower = (rotatedY - rotatedX - rightStickX) * normalize;
        fcPowerLevels.backRightPower = (rotatedY + rotatedX - rightStickX) * normalize;
    }

    // Class to hold field centric power level output from getFieldCentricPowerLevels

}
