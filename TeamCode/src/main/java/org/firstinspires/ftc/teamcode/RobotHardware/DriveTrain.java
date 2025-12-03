package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;

public class DriveTrain {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;
    public FieldCentricPowerLevels fcPowerLevels = new FieldCentricPowerLevels();
    private final AndroidLog log = new AndroidLog();
    public DriveTrain(HardwareMap hardwareMap)
    {

        // Drive Motors
        // Names need to match configuration on driver hub
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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
        log.d("PathController", "===========================================");
        log.d("PathController", String.format("fl: %f", fcPowerLevels.frontLeftPower));
        log.d("PathController", String.format("fr: %f", fcPowerLevels.frontRightPower));
        log.d("PathController", String.format("bl: %f", fcPowerLevels.backLeftPower));
        log.d("PathController", String.format("br: %f", fcPowerLevels.backRightPower));
        log.d("PathController", "===========================================");
        // The power scale has to be negative for two of them because their problematic
        frontLeft.setPower(fcPowerLevels.frontLeftPower * powerScale);
        backLeft.setPower(fcPowerLevels.backLeftPower * -powerScale);
        frontRight.setPower(fcPowerLevels.frontRightPower * -powerScale);
        backRight.setPower(fcPowerLevels.backRightPower * powerScale);
    }
    private void getFieldCentricPowerLevels(
            double drive, double strafe,
            double rotate, double botHeading) {

        // drive forward/backward [-1.0, 1.0]
        // strafe strafe [-1.0, 1.0]
        // rotate rotate [-1.0, 1.0].

        // Rotate joystick input vectors for field centric control.

        // Apply rotation matrix inverse of botHeading (-botHeading).
        // | cos(-h) -sin(-h) | | lsx  lsy |
        // | sin(-h)  cos(-h) | | lsx  lsy |
        // h = botHeading
        // lsx = left stick x value
        // lsy = left stick y value.
        double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // Don't know why the rotated value of left stick x is scale by 1.1.
        // It's not in online example code.
        // rotatedX = rotatedX * 1.1;

        // Normalize output power [-1.0-1.0]
        double vectorSum = Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rotate);
        double normalize = Math.max(vectorSum, 1.0);

        // Set normalized field centric power levels.
        fcPowerLevels.frontLeftPower = -(rotatedY + rotatedX - rotate) * normalize;
        fcPowerLevels.backLeftPower = (rotatedY - rotatedX - rotate) * normalize;
        fcPowerLevels.frontRightPower = (rotatedY - rotatedX + rotate) * normalize;
        fcPowerLevels.backRightPower = -(rotatedY + rotatedX + rotate) * normalize;
    }

    // Class to hold field centric power level output from getFieldCentricPowerLevels

}
