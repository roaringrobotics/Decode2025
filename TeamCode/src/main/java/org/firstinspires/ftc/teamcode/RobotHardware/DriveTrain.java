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
// java
    public void driveStraight(double distance, double power, ImuPositionI imu, LogI log,
                              double kP, double kI, double kD) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();
        double startHeadingDeg = imu.getHeading(AngleUnit.DEGREES);

        // Target pose (used only for computing delta to final point)
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + distance * Math.cos(Math.toRadians(startHeadingDeg)),
                startPose.getY(DistanceUnit.INCH) + distance * Math.sin(Math.toRadians(startHeadingDeg)),
                AngleUnit.DEGREES, startHeadingDeg);

        PID pid = new PID(kP, kI, kD);
        final double distanceTolerance = 0.25; // inches
        final double minDrivePower = 0.15;     // minimum drive power to overcome static friction
        final int settleCountsRequired = 10;

        // Small PD for heading correction
        final double headingKp = 0.02;
        final double headingKd = 0.002;
        double lastHeadingError = 0.0;

        int settleCount = 0;
        double distanceTraveled = 0.0;
        double delta = Vector2.distanceBetweenPoses(targetPose, startPose); // initial delta ~= abs(distance)

        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = imu.getHeading(AngleUnit.DEGREES);

            distanceTraveled = Vector2.distanceBetweenPoses(startPose, curPose);
            // remaining distance to target
            delta = Vector2.distanceBetweenPoses(targetPose, curPose);

            // Determine motion direction sign (forward/backward)
            // If requested distance was negative, we should drive backwards.
            double sign = distance >= 0 ? 1.0 : -1.0;
            double remaining = Math.abs(distance) - distanceTraveled;

            // Check settle condition
            if (Math.abs(remaining) <= distanceTolerance) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // PID: many project PID.calculate(setpoint, current)
            double basePower = pid.calculate(Math.abs(distance), distanceTraveled);
            // Ensure signed power for forward/backward
            basePower = basePower * sign;

            // Apply minimum drive power only when still away from target
            if (Math.abs(remaining) > distanceTolerance && Math.abs(basePower) < minDrivePower) {
                basePower = minDrivePower * Math.signum(basePower != 0.0 ? basePower : sign);
            }

            // Heading correction to keep straight using initial heading
            double headingError = startHeadingDeg - curHeading;
            // normalize to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double headingDerivative = (headingError - lastHeadingError) / dt;
            lastHeadingError = headingError;

            double headingCorrection = headingKp * headingError + headingKd * headingDerivative;
            // Apply heading correction to left/right (subtract from left, add to right for correction)
            double leftPower = basePower - headingCorrection;
            double rightPower = basePower + headingCorrection;

            // Clamp powers
            leftPower = Math.max(-power, Math.min(power, leftPower));
            rightPower = Math.max(-power, Math.min(power, rightPower));

            // Set motor powers (use set* methods so motor direction config is respected)
            setFrontLeftPower(leftPower);
            setBackLeftPower(leftPower);
            setFrontRightPower(rightPower);
            setBackRightPower(rightPower);

            if (log != null) {
                log.d("DriveStraight", String.format("rem: %.3f trav: %.3f base: %.3f L: %.3f R: %.3f hErr: %.2f",
                        remaining, distanceTraveled, basePower, leftPower, rightPower, headingError));
            }

            // Loop cadence
            sleep(20);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveStraight", "Complete. Traveled: " + distanceTraveled);
        }
    }
    public void driveLinear(double x,
                            double y,
                            double endDegrees,
                            double power,
                            ImuPositionI imu,
                            LogI log) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();

        // Calculate target pose in field coordinates
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + x,
                startPose.getY(DistanceUnit.INCH) + y,
                AngleUnit.DEGREES,
                endDegrees);

        double targetDistance = Math.sqrt(x * x + y * y);

        // PID controllers
        PID drivePID = new PID(0.1, 0.0, 0.01);     // For position control
        PID rotatePID = new PID(0.012, 0.0001, 0.001); // For rotation control

        final double distanceTolerance = 0.5;  // inches
        final double angleTolerance = 2.0;     // degrees
        final double minDrivePower = 0.12;
        final double minRotatePower = 0.15;
        final int settleCountsRequired = 10;

        int settleCount = 0;
        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = imu.getHeading(AngleUnit.DEGREES);

            // Calculate remaining distance to target
            double deltaX = targetPose.getX(DistanceUnit.INCH) - curPose.getX(DistanceUnit.INCH);
            double deltaY = targetPose.getY(DistanceUnit.INCH) - curPose.getY(DistanceUnit.INCH);
            double remainingDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Calculate heading error
            double headingError = endDegrees - curHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Check if we're on target
            boolean positionOnTarget = remainingDistance <= distanceTolerance;
            boolean headingOnTarget = Math.abs(headingError) <= angleTolerance;

            if (positionOnTarget && headingOnTarget) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // Calculate drive vector in field frame
            double driveX = 0.0;
            double driveY = 0.0;

            if (!positionOnTarget) {
                // PID control based on remaining distance
                double driveMagnitude = drivePID.calculate(targetDistance, targetDistance - remainingDistance);

                // Apply minimum power
                if (Math.abs(driveMagnitude) < minDrivePower) {
                    driveMagnitude = minDrivePower * Math.signum(driveMagnitude);
                }

                // Direction vector (normalized)
                double dirX = deltaX / remainingDistance;
                double dirY = deltaY / remainingDistance;

                driveX = dirX * driveMagnitude;
                driveY = dirY * driveMagnitude;
            }

            // Calculate rotation power
            double rotatePower = 0.0;
            if (!headingOnTarget) {
                rotatePower = rotatePID.calculate(endDegrees, curHeading);

                // Apply minimum rotation power
                if (Math.abs(rotatePower) < minRotatePower && Math.abs(headingError) > angleTolerance) {
                    rotatePower = minRotatePower * Math.signum(rotatePower);
                }
            }

            // Convert field-centric to robot-centric
            // Rotate drive vector by -botHeading to get robot-relative motion
            double botHeadingRad = Math.toRadians(curHeading);
            double robotX = driveX * Math.cos(-botHeadingRad) - driveY * Math.sin(-botHeadingRad);
            double robotY = driveX * Math.sin(-botHeadingRad) + driveY * Math.cos(-botHeadingRad);

            // Apply the mysterious 1.1 scaling factor for strafe (from your original code)
            robotX = robotX * 1.1;

            // Calculate mecanum wheel powers
            double vectorSum = Math.abs(robotY) + Math.abs(robotX) + Math.abs(rotatePower);
            double denom = Math.max(vectorSum, 1.0);

            double flPower = (robotY + robotX + rotatePower) / denom;
            double blPower = (robotY - robotX + rotatePower) / denom;
            double frPower = (robotY - robotX - rotatePower) / denom;
            double brPower = (robotY + robotX - rotatePower) / denom;

            // Apply power limit and signs (matching your motor configuration)
            setFrontLeftPower(flPower * -power);
            setBackLeftPower(-blPower * -power);    // Note the negative
            setFrontRightPower(-frPower * -power);   // Note the negative
            setBackRightPower(brPower * -power);

            if (log != null) {
                log.d("DriveLinear", String.format("Dist: %.2f Angle: %.1f | FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                        remainingDistance, headingError, flPower, frPower, blPower, brPower));
            }

            sleep(20);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveLinear", "Complete - reached target position and heading");
        }
    }


    public void rotate(double targetAngleDeg, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        double heading = imu.getHeading(AngleUnit.DEGREES);

        double error = targetAngleDeg - heading;
        // normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        PID pidRotate = new PID(0.012, 0.0001, 0.001); // Example tuned values: adjust Kp/Ki/Kd based on testing
        double minPower = 0.18; // Minimum power to overcome friction (tune this!)
        int onTargetCount = 0;
        int requiredCounts = 10; // Must be on-target for ~10 loops

        while (Math.abs(error) > 2 || onTargetCount < requiredCounts) { // Coarse + settle
            double power = pidRotate.calculate(0, error); // Many PID impls use calculate(setpoint=0, processVar=error)
            // Or if yours is calculate(target, current): pidRotate.calculate(targetAngleDeg, heading)

            // Apply min power (preserve sign)
            if (Math.abs(power) < minPower && Math.abs(error) > 2) {
                power = minPower * Math.signum(power);
            }

            // Clamp
            power = Math.max(-1.0, Math.min(1.0, power));

            // Tank rotate in place (assuming standard FTC config: +power = clockwise turn)
            setFrontLeftPower(-power);
            setBackLeftPower(-power);
            setFrontRightPower(power);
            setBackRightPower(power);

            log.d("heading", String.valueOf(heading));
            log.d("error", String.valueOf(error));
            log.d("power", String.valueOf(power));
            log.d("", "----------------------------");

            imu.update();
            heading = imu.getHeading(AngleUnit.DEGREES);

            error = targetAngleDeg - heading;
            // normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            if (Math.abs(error) < 2) onTargetCount++;
            else onTargetCount = 0;

            sleep(20); // Slower loop for stability
        }

        stopMotors();
    }
    public static double clamp(double value, double min, double max) {
        if (min > max) throw new IllegalArgumentException("min must be <= max");
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Rotates the robot in place by a relative angle (positive = clockwise, negative = counterclockwise).
     * Uses PID control for smooth and accurate turns.
     *
     * @param relativeAngleDeg The angle to rotate relative to current heading (e.g., +90 for 90° right)
     * @param imu              IMU interface for getting current heading
     * @param log              Logger for debugging (optional, can be null if not needed)
     * @throws Exception
     */
    public void rotateRelative(double relativeAngleDeg, ImuPositionI imu, LogI log) throws Exception {
        imu.update();
        double currentHeading = imu.getHeading(AngleUnit.DEGREES);

        // Calculate absolute target heading
        double targetHeading = currentHeading + relativeAngleDeg;

        // Normalize target to [-180, 180] or [0, 360] — here we keep it continuous but normalize error later
        // No need to normalize target itself, only the error

        PID pid = new PID(0.01, 0, 0); // Tune these: Kp, Ki, Kd
        double minPower = 0.18;       // Minimum power to overcome friction — adjust based on your robot
        double toleranceDeg = 2.0;    // Consider "on target" if within this
        int settleCountsRequired = 15; // Must stay within tolerance for ~15 loops (~300ms at 20ms loop)
        int settleCount = 0;

        while (true) {
            imu.update();
            currentHeading = imu.getHeading(AngleUnit.DEGREES);

            // Calculate error with shortest path (-180 to +180)
            double error = targetHeading - currentHeading;
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            // Exit condition: small error and settled
            if (Math.abs(error) <= toleranceDeg) {
                settleCount++;
                if (settleCount >= settleCountsRequired) {
                    break;
                }
            } else {
                settleCount = 0;
            }

            // PID calculation — assuming your PID.calculate(setpoint, current) returns correction
            // Many FTC PID classes use calculate(target, current), so:
            double power = pid.calculate(targetHeading, currentHeading);

            // Alternative: if your PID expects error directly or setpoint=0:
            // double power = pid.calculate(0, error);

            // Apply minimum power when needed (preserve direction)
            if (Math.abs(power) < minPower && Math.abs(error) > toleranceDeg) {
                power = minPower * Math.signum(power);
            }

            // Clamp power to motor limits
            power = Math.max(-1.0, Math.min(1.0, power));

            // Apply tank turn: positive power = clockwise rotation (common convention)
            setFrontLeftPower(-power);
            setBackLeftPower(-power);
            setFrontRightPower(power);
            setBackRightPower(power);

            // Optional logging
            if (log != null) {
                log.d("RotateRel", "Target: " + String.format("%.1f", targetHeading) +
                        " | Current: " + String.format("%.1f", currentHeading) +
                        " | Error: " + String.format("%.1f", error) +
                        " | Power: " + String.format("%.3f", power));
            }

            sleep(20); // 50 Hz loop — stable and efficient
        }

        stopMotors();

        if (log != null) {
            log.d("RotateRel", "Rotation complete. Final heading: " + currentHeading);
        }
    }

    public void driveStrafe(double distance, double power, ImuPositionI imu, LogI log,
                              double kP, double kI, double kD) throws Exception {
        imu.update();
        Pose2D startPose = imu.getPose();
        double startHeadingDeg = imu.getHeading(AngleUnit.DEGREES);

        // Target pose (used only for computing delta to final point)
        Pose2D targetPose = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(DistanceUnit.INCH) + distance * Math.cos(Math.toRadians(startHeadingDeg)),
                startPose.getY(DistanceUnit.INCH) + distance * Math.sin(Math.toRadians(startHeadingDeg)),
                AngleUnit.DEGREES, startHeadingDeg);

        PID pid = new PID(kP, kI, kD);
        final double distanceTolerance = 0.25; // inches
        final double minDrivePower = 0.15;     // minimum drive power to overcome static friction
        final int settleCountsRequired = 10;

        // Small PD for heading correction
        final double headingKp = 0.02;
        final double headingKd = 0.002;
        double lastHeadingError = 0.0;

        int settleCount = 0;
        double distanceTraveled = 0.0;
        double delta = Vector2.distanceBetweenPoses(targetPose, startPose); // initial delta ~= abs(distance)

        long lastTime = System.nanoTime();

        while (true) {
            imu.update();
            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastTime) / 1e9);
            lastTime = now;

            Pose2D curPose = imu.getPose();
            double curHeading = imu.getHeading(AngleUnit.DEGREES);

            distanceTraveled = Vector2.distanceBetweenPoses(startPose, curPose);
            // remaining distance to target
            delta = Vector2.distanceBetweenPoses(targetPose, curPose);

            // Determine motion direction sign (forward/backward)
            // If requested distance was negative, we should drive backwards.
            double sign = distance >= 0 ? 1.0 : -1.0;
            double remaining = Math.abs(distance) - distanceTraveled;

            // Check settle condition
            if (Math.abs(remaining) <= distanceTolerance) {
                settleCount++;
                if (settleCount >= settleCountsRequired) break;
            } else {
                settleCount = 0;
            }

            // PID: many project PID.calculate(setpoint, current)
            double basePower = pid.calculate(Math.abs(distance), distanceTraveled);
            // Ensure signed power for forward/backward
            basePower = basePower * sign;

            // Apply minimum drive power only when still away from target
            if (Math.abs(remaining) > distanceTolerance && Math.abs(basePower) < minDrivePower) {
                basePower = minDrivePower * Math.signum(basePower != 0.0 ? basePower : sign);
            }

            // Heading correction to keep straight using initial heading
            double headingError = startHeadingDeg - curHeading;
            // normalize to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double headingDerivative = (headingError - lastHeadingError) / dt;
            lastHeadingError = headingError;

            double headingCorrection = headingKp * headingError + headingKd * headingDerivative;
            // Apply heading correction to left/right (subtract from left, add to right for correction)
            double leftPower = basePower - headingCorrection;
            double rightPower = basePower + headingCorrection;

            // Clamp powers
            leftPower = Math.max(-power, Math.min(power, leftPower));
            rightPower = Math.max(-power, Math.min(power, rightPower));

            // Set motor powers (use set* methods so motor direction config is respected)
            setFrontLeftPower(leftPower);
            setBackLeftPower(-leftPower);
            setFrontRightPower(-rightPower);
            setBackRightPower(rightPower);

            if (log != null) {
                log.d("DriveStraight", String.format("rem: %.3f trav: %.3f base: %.3f L: %.3f R: %.3f hErr: %.2f",
                        remaining, distanceTraveled, basePower, leftPower, rightPower, headingError));
            }

            // Loop cadence
            sleep(20);
        }

        stopMotors();
        if (log != null) {
            log.d("DriveStrafe", "Complete. Traveled: " + distanceTraveled);
        }
    }
    public double powerScaler(boolean halfSpeed, boolean quarterSpeed) {
        double powerScale;
        if (halfSpeed) {
            powerScale = 0.5;
        } else if (quarterSpeed) {
            powerScale = 0.25;
        } else {
            powerScale = 1.0;
        }
        return powerScale;
    }

}

