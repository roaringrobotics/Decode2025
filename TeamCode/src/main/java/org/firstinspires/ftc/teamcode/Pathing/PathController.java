package org.firstinspires.ftc.teamcode.Pathing;

import static java.lang.Math.toDegrees;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;
import org.firstinspires.ftc.teamcode.Math.RotationMatrix2d;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Interfaces.OpModeI;
import org.firstinspires.ftc.teamcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Interfaces.HardwareI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

public class PathController {

    private final TimeSourceI timeSource;
    private final LogI log;

    public PathController(
            HardwareI hw, OpModeI opMode, double nominalPower,
            PidParamCollection.ParamSetName pidParams,
            TimeSourceI ts, LogI logInterface) {
        this.hardWare = hw;
        this.opMode = opMode;
        this.nominalPower = nominalPower;
        this.timeSource = ts;
        this.log = logInterface;

        powerRampControlFl = new PowerRampController(.1, timeSource);
        powerRampControlFr = new PowerRampController(.1, timeSource);
        powerRampControlBl = new PowerRampController(.1, timeSource);
        powerRampControlBr = new PowerRampController(.1, timeSource);

        PidParamSet hps = PidParamCollection.paramsSets.get(PidParamCollection.ParamSetName.UNIT_TEST_SIM);
        assert hps != null;
        headingPid = new PIDController(hps.heading);
        deltaTargetPid = new PIDController(hps.location);
    }

    public void setNominalPower(double power)
    {
        this.nominalPower = power;
    }

    @NonNull
    private Vector2 computeMovementVector(Vector2 targetLocation) {
        lastPose = hardWare.getImuPose();
        return new Vector2(0, 0);
    }

    public void setTargetHeadingDeg(double newTargetHeading) {
        // [180, -180)
        // Positive is clockwise
        targetHeadingDeg = newTargetHeading;
    }

    public void setTargetLocation(int x, int y) {
        // Drive to relative x,y (inches) position and maintain/rotate to targetHeadingDeg.
        // Coordinate system of play field based on initial position of robot.
        // R = robot, ^ = initial facing position of robot.
        //         +Y
        //          ^  ^
        //          |  R
        //-X <------|------>+X
        //          |
        //          \/
        //         -Y

        this.targetLocation.x = x;
        this.targetLocation.y = y;
    }

    public boolean compareHeading(double rh, double lh)
    {
        // compare heading to 1/4 degree.
        return Math.abs(lh - rh) < .25;
    }

    public boolean notAtTarget()
    {
        // Test is not at target heading and loc.
        double closeEnoughInches = 0.05;
        Pose2D currentPose = hardWare.getImuPose();
        double delta = Vector2.deltaNorm(currentPose, targetLocation);
        boolean atLoc = delta < closeEnoughInches;
        boolean atHeading = compareHeading(currentPose.getHeading(AngleUnit.DEGREES), targetHeadingDeg);
        return !(atLoc && atHeading);
    }

    public void stop()
    {
        // Turn off motors.
        // Call drive again to resume.
        hardWare.setFrontLeftPower(0.0);
        hardWare.setFrontRightPower(0.0);
        hardWare.setBackLeftPower(0.0);
        hardWare.setBackRightPower(0.0);
    }

    public void drive() {
        // Takes current heading and position
        // and adjusts motor power to drive towards target x,y and targetHeadingDeg.

        // This gets the current x,y and normalized [180,-180) heading
        hardWare.updateImuPos();
        lastPose = hardWare.getImuPose();

        // Compute normalize vector pointing to target location.
        motionVector.subtractInPlace(targetLocation,lastPose);
        motionVector.normalize();

        // Based on how far the
        double deltaToTarget = Vector2.deltaNorm(lastPose, targetLocation);
        // Need to invert the pid value here since we are using using delta instead of position.
        double deltaTargetPidValue = -deltaTargetPid.calculate(deltaToTarget, timeSource);
        log.d("drive", String.format("deltaTargetPidValue: %f", deltaTargetPidValue));
        deltaTargetPidValue = clampRange(deltaTargetPidValue, -0.9, 0.9);
        motionVector.scale(nominalPower*deltaTargetPidValue);

        // Rotate the vector based on robot heading to
        // get the power vector needed to drive that direction.
        double heading = lastPose.getHeading(AngleUnit.RADIANS);
        robotToFieldRotation.setRotation(-heading, AngleUnit.RADIANS);
        robotRelativePowerVector = robotToFieldRotation.multiply(motionVector);

        // Calculate rotation correction using PID.
        headingPid.setTargetPoint(targetHeadingDeg);
        rotationScalar = headingPid.calculate(toDegrees(heading), timeSource);

        // Clamp magnitude of rotationScalar [-0.3, 0.3]
        rotationScalar = clampRange(rotationScalar, -0.3, 0.3);

        // Normalize output power [-1.0-1.0]
        vectorSum = Math.abs(robotRelativePowerVector.x) + Math.abs(robotRelativePowerVector.y) + Math.abs(rotationScalar);
        normalize = 1.0 / Math.max(vectorSum, 1.0);

        // Set normalized field centric power levels.
        // Clamp to valid range [-1,1] for motors.
        frontLeftPower = motorClamp((robotRelativePowerVector.y + robotRelativePowerVector.x + rotationScalar) * normalize);
        backLeftPower = motorClamp((robotRelativePowerVector.y - robotRelativePowerVector.x + rotationScalar) * normalize);
        frontRightPower = motorClamp((robotRelativePowerVector.y - robotRelativePowerVector.x - rotationScalar) * normalize);
        backRightPower = motorClamp((robotRelativePowerVector.y + robotRelativePowerVector.x - rotationScalar) * normalize);

        // set power to the real motors.
        setMotorPower();
    }

    private double motorClamp(double v) {
        // Clamp value to range valid for motors.
        return clampRange(v, -1.0, 1.0);
    }

    double vectorSum;
    double normalize;

    private void setMotorPower()
    {
        double fl = powerRampControlFl.getValue(frontLeftPower);
        double fr = powerRampControlFr.getValue(frontRightPower);
        double bl = powerRampControlBl.getValue(backLeftPower);
        double br = powerRampControlBr.getValue(backRightPower);

        hardWare.setFrontLeftPower(fl);
        hardWare.setFrontRightPower(fr);
        hardWare.setBackLeftPower(bl);
        hardWare.setBackRightPower(br);
    }

    private double clampRange(double value, double min, double max) {
        // clamp value into range [min, max]
        double clamped = value;
        if(value < min)
            clamped = min;
        else if(value > max)
            clamped = max;
        return clamped;
    }

    public void updateTelemetry()
    {
        TelemetryHelper.UpdateTelemetry(opMode.getTelemetry(),
                "currentLoc x:", lastPose.getX(DistanceUnit.INCH),
                "currentLoc y:", lastPose.getY(DistanceUnit.INCH),
                "currentHeading:", lastPose.getHeading(AngleUnit.DEGREES),
                "getheading:", hardWare.getImuHeading(AngleUnit.DEGREES),
                "delta Target:", Vector2.deltaNorm(lastPose, targetLocation),
                "rotscaler:", rotationScalar,
                "vecsum:", vectorSum,
                "normalize:", normalize,
                "fl power", hardWare.getFrontLeftPower(),
                "fr power", hardWare.getFrontRightPower(),
                "bl power", hardWare.getBackLeftPower(),
                "br power", hardWare.getBackRightPower());
    }


    public boolean run() {
        // Runs until stop or at target.
        return run(10);
    }
    public boolean run(long timeoutSec) {
        // Runs until stop or at target or timeout reached.
        while(notAtTarget() && !opMode.isStopRequested()) {
            timeSource.update();
            hardWare.updateState(timeSource); // used for simulation
            opMode.updateState(timeSource); // used for simulation
            drive();
            updateTelemetry();

            if(timeSource.totalRunningTimeMs()/1000 > timeoutSec) {
                log.d("PathController", "run time: " + timeSource.totalRunningTimeMs());
                hardWare.stopMotors();
                return false;
            }
        }
        log.d("PathController", "run time: " + timeSource.totalRunningTimeMs());
        hardWare.stopMotors();
        return true;
    }

    public void setRotPidCoeff(double kp, double ki, double kd) {
        this.headingPid.setCoeff(kp,ki,kd);
    }

    private final OpModeI opMode;
    private final HardwareI hardWare;

    double targetHeadingDeg;
    Vector2 targetLocation = new Vector2(0,0);
    double rotationScalar;
    Vector2 motionVector = new Vector2(0,0);
    double nominalPower;

    Pose2D lastPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    RotationMatrix2d robotToFieldRotation = new RotationMatrix2d();
    Vector2 robotRelativePowerVector = new Vector2(0,0);

    PidParamSet pidParams;
    PIDController headingPid;
    PIDController deltaTargetPid;
    PowerRampController powerRampControlFl;
    PowerRampController powerRampControlFr;
    PowerRampController powerRampControlBl;
    PowerRampController powerRampControlBr;
}
