package org.firstinspires.ftc.teamcode.RobotHardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Pathing.PID;

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
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        frontLeft.setPower(power);
    }

    public void setFrontRightPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        frontRight.setPower(power);

    }

    public void setBackLeftPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
        backLeft.setPower(power);

    }

    public void setBackRightPower(double power) {
        power = Math.min(power, 1.0);
        power = Math.max(power, -1.0);
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

    /**
     * Adjusts motor powers to follow a target heading. Call repeatedly from a loop.
     * @param imu IMU implementation providing heading (degrees)
     * @param targetAngleDeg target heading in degrees
     * @param maxPower maximum absolute motor power (0..1)
     * @param kP proportional gain applied to heading error
     * @param toleranceDeg if absolute error is within this, motors are stopped
     */
    public void followHeading(ImuPositionI imu,
                              double targetAngleDeg,
                              double maxPower,
                              double kP,
                              double toleranceDeg) {
        imu.update();
        double heading = imu.getHeading(AngleUnit.DEGREES);

        double error = targetAngleDeg - heading;
        // normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        if (Math.abs(error) <= toleranceDeg) {
            stopMotors();
            return;
        }

        double turn = kP * error;
        if (turn > maxPower) turn = maxPower;
        if (turn < -maxPower) turn = -maxPower;

        // Apply turn power: left motors positive, right motors negative (matches driveFieldCentric convention)
        frontLeft.setPower(turn);
        backLeft.setPower(turn);
        frontRight.setPower(-turn);
        backRight.setPower(-turn);
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
//        log.d("PathController", "===========================================");
//        log.d("PathController", String.format("fl: %f", fcPowerLevels.frontLeftPower));
//        log.d("PathController", String.format("fr: %f", fcPowerLevels.frontRightPower));
//        log.d("PathController", String.format("bl: %f", fcPowerLevels.backLeftPower));
//        log.d("PathController", String.format("br: %f", fcPowerLevels.backRightPower));
//        log.d("PathController", "===========================================");

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
        rotatedX = rotatedX * 1.1;

        // Normalize output power [-1.0-1.0]
        double vectorSum = Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rotate);
        double denom = Math.max(vectorSum, 1.0);

        // Set normalized field centric power levels.
        fcPowerLevels.frontLeftPower = -(rotatedY + rotatedX - rotate) / denom;
        fcPowerLevels.backLeftPower = (rotatedY - rotatedX - rotate) / denom;
        fcPowerLevels.frontRightPower = (rotatedY - rotatedX + rotate) / denom;
        fcPowerLevels.backRightPower = -(rotatedY + rotatedX + rotate) / denom;
    }

    // Class to hold field centric power level output from getFieldCentricPowerLevels
    public void driveStraight(double distance, double power, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();
        double delta = distance;
        do {

            setFrontRightPower(power);
            setFrontLeftPower(power);
            setBackRightPower(power);
            setBackLeftPower(power);

            sleep(1);
            imu.update();
            Pose2D curPose = imu.getPose();

            delta = Vector2.distanceBetweenPoses(startPose, curPose);

             log.d("DriveStraight", String.format("Delta: %f", delta));
        } while(delta < Math.abs(distance));

        stopMotors();
    }

    public void rotate(double deltaAngle, double maxPower, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        double heading = imu.getHeading(AngleUnit.DEGREES);
        double startHeading = heading;
        double sign = 1;

        PID pidRotate = new PID(0.065, 0, 0);
        if (deltaAngle < 0) {;
            sign = -1;
        }
        deltaAngle = Math.abs(deltaAngle);
        double deltaHeading = Math.abs(heading) - startHeading;

        while (deltaAngle > deltaHeading) {
            double power = clamp(pidRotate.calculate(deltaAngle, Math.abs(heading)), -maxPower, maxPower) * sign;


            log.d("turning", "Delta Angle: " + deltaAngle);
            log.d("turning", "Heading: " + heading);
            log.d("turning", "Delta Heading: " + deltaHeading);
            log.d("turning", "power: " + power);

            setFrontLeftPower(power);
            setBackLeftPower(power);
            setFrontRightPower(-power);
            setBackRightPower(-power);

            imu.update();
            heading = imu.getHeading(AngleUnit.DEGREES);
            deltaHeading = Math.abs(heading) - startHeading;
        }
        log.d("turning", "Final Position");
        log.d("turning", "Delta Angle: " + deltaAngle);
        log.d("turning", "Heading: " + heading);
        log.d("turning", "Delta Heading: " + deltaHeading);


    }

    public void rotate2(double targetAngleDeg, double maxPower, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        double heading = imu.getHeading(AngleUnit.DEGREES);
        double startHeading = heading;

        double error = targetAngleDeg - heading;
        // normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        PID pidRotate = new PID(0.09, 0.01, 0);
        while (Math.abs(error) > 10) {
            double power = clamp(pidRotate.calculate(targetAngleDeg, heading), -maxPower, maxPower);
            log.d("heading", String.valueOf(heading));
            log.d("power", String.valueOf(power));
            log.d("", "----------------------------");
            if (error > 0) {
                setFrontLeftPower(-power);
                setBackLeftPower(-power);
                setFrontRightPower(power);
                setBackRightPower(power);
            } else {
                setFrontLeftPower(power);
                setBackLeftPower(power);
                setFrontRightPower(-power);
                setBackRightPower(-power);
            }

            imu.update();
            heading = imu.getHeading(AngleUnit.DEGREES);

            error = targetAngleDeg - heading;
            // normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            sleep(1);
        }

        stopMotors();
    }
    public static double clamp(double value, double min, double max) {
        if (min > max) throw new IllegalArgumentException("min must be <= max");
        return Math.max(min, Math.min(max, value));
    }


}
