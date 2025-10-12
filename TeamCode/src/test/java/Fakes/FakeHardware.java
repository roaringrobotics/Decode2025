package Fakes;

import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Interfaces.HardwareI;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;
import org.firstinspires.ftc.teamcode.Math.Matrix2;
import org.firstinspires.ftc.teamcode.Math.Vector2;

// Simple robot drive train simulation for unit testing.
// I'm sure a better simulation of Mecanum drive is available somewhere.
public class FakeHardware implements HardwareI {

    LogI logger = new FakeLog();
    @Override
    public void updateImuPos() {

    }

    @Override
    public void resetImu() {

    }

    @Override
    public void resetImuHeading() {

    }

    @Override
    public Pose2D getImuPose() {
        return new Pose2D(DistanceUnit.INCH, location.x, location.y, AngleUnit.DEGREES, getImuHeading(AngleUnit.DEGREES));
    }

    @Override
    public double getImuHeading(AngleUnit unit) {
        double heading;
        if(unit == AngleUnit.DEGREES)
            heading = toDegrees(imuHeading);
        else
            heading = imuHeading;

        return heading;
    }

    @Override
    public void setFrontLeftPower(double power) {
        frontLeftPower = power;
    }

    @Override
    public void setFrontRightPower(double power) {
        frontRightPower = power;
    }

    @Override
    public void setBackLeftPower(double power) {
        backLeftPower = power;
    }

    @Override
    public void setBackRightPower(double power) {
        backRightPower = power;
    }

    @Override
    public double getFrontLeftPower() {
        return frontLeftPower;
    }

    @Override
    public double getFrontRightPower() {
        return frontRightPower;
    }

    @Override
    public double getBackLeftPower() {
        return backLeftPower;
    }

    @Override
    public double getBackRightPower() {
        return backRightPower;
    }

    @Override
    public void updateState(TimeSourceI timeSource) {
        // Update for simulation during testing.

        long deltaTimeMs = timeSource.deltaTimeMs();
        double deltaTimeSec = Constants.millisecondsToSeconds((double) deltaTimeMs);

        // Update speed scaler for each wheel based on set power.
        double speedFrontLeft = maxSpeedInchSec * frontLeftPower;
        double speedFrontRight = maxSpeedInchSec * frontRightPower;
        double speedBackLeft = maxSpeedInchSec * backLeftPower;
        double speedBackRight = maxSpeedInchSec * backRightPower;

        logger.d("FakeHardware", "speedFrontLeft: " + speedFrontLeft);
        logger.d("FakeHardware", "speedFrontRight: " + speedFrontRight);
        logger.d("FakeHardware", "speedBackLeft: " + speedBackLeft);
        logger.d("FakeHardware", "speedBackRight: " + speedBackRight);

        // Update the velocity vector.  Sum of the wheel direction unit vectors
        // times speed.

        // Compute arc along wheel track circle the robot will
        // rotate given the difference in speed between right and left sides.
        Vector2 leftVelocity = Vector2.add(Vector2.mult(flWheelDv, speedFrontLeft), Vector2.mult(blWheelDv, speedBackLeft));
        leftVelocity.scale(.5);
        Vector2 rightVelocity = Vector2.add(Vector2.mult(frWheelDv, speedFrontRight), Vector2.mult(brWheelDv, speedBackRight));
        rightVelocity.scale(.5);

        // Only care about y components since left and right vector sums are tangent
        // in the y direction to the robots center of rotation.
        double tangentialVelocity = (leftVelocity.y - rightVelocity.y) * 0.5;
        double arclengthTraveled = tangentialVelocity * deltaTimeSec;
        // convert the length to radians
        double deltaAngleRad = 2 * Math.PI * (arclengthTraveled / wheelTrackRadiusInches);
        double currentHeadingRad = getImuHeading(AngleUnit.RADIANS);
        imuHeading = currentHeadingRad + deltaAngleRad;
        rotation.setRotation(imuHeading, AngleUnit.RADIANS);
        double headingDeg = toDegrees(imuHeading);
        logger.d("FakeHardware", "imuHeading: " + headingDeg);
        logger.d("FakeHardware", "leftVelocity: " + leftVelocity);
        logger.d("FakeHardware", "rightVelocity: " + rightVelocity);

        velocityInchesSec = Vector2.add(leftVelocity, rightVelocity);
        velocityInchesSec.scale(.5);
        // Rotate the velocity
        velocityInchesSec = rotation.mult(velocityInchesSec);

        // Update the location based on the speed and delta time.
        location = Vector2.add(location, Vector2.mult(velocityInchesSec, deltaTimeSec));
        logger.d("fakeHardware","velocityInchesSec: " + velocityInchesSec.toString());
        logger.d("fakeHardware","location: " + location.toString());
    }

    @Override
    public void stopMotors() {
        setFrontLeftPower(0);
        setFrontRightPower(0);
        setBackLeftPower(0);
        setBackRightPower(0);
    }

    // direction and speed of simulated robot.  Inches/second.
    Vector2 velocityInchesSec = new Vector2(0, 0);

    // Current location.
    Vector2 location = new Vector2(0,0);

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    // Direction unit vectors for Mecanum wheels rotation forward.
    final Vector2 flWheelDv = new Vector2(0.7071, 0.7071);
    final Vector2 frWheelDv = new Vector2(-0.7071, 0.7071);
    final Vector2 blWheelDv = new Vector2(-0.7071, 0.7071);
    final Vector2 brWheelDv = new Vector2(0.7071, 0.7071);

    double maxSpeedInchSec = 12;

    // Distance from center of robot to wheels.
    // This is also the arc length per radian used
    // to simulate rotation of the robot given different wheel speeds.
    double wheelTrackRadiusInches = 8;

    Matrix2 rotation = new Matrix2();

    double imuHeading = 0;
}
