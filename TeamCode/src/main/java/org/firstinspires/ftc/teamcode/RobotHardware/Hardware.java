package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Interfaces.HardwareI;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

// Initializes and tracks the Robot Hardware.
public class Hardware implements HardwareI {

    public Hardware(HardwareMap hardwareMap) throws InterruptedException {
        InitDriveMotors(hardwareMap);
//        InitLiftMotors(hardwareMap);
//        InitExtendServos(hardwareMap);
//        InitGripServos(hardwareMap);
        InitImu(hardwareMap);
    }

    private void InitImu(HardwareMap hardwareMap) {
        // Initialize IMU and Encoders.  imuPos can be internal IMU/Encoders or Pinpoint computer.
        // imuPos = new ImuImp(hardwareMap);
        imuPos = new PinpointImpl(hardwareMap); // use pinpoint
    }

    private void InitGripServos(HardwareMap hardwareMap) {
        // Grip servos
        // Names need to match configuration on driver hub
        blackGrip = hardwareMap.servo.get("blackGrip");
        blueGrip = hardwareMap.servo.get("blueGrip");
    }

    private void InitExtendServos(HardwareMap hardwareMap) {
        // Extender servos
        // Names need to match configuration on driver hub
        blackExtend = hardwareMap.servo.get("blackExtend");
        blueExtend = hardwareMap.servo.get("blueExtend");
    }

    private void InitLiftMotors(HardwareMap hardwareMap) {
        // Lift motors
        // Names need to match configuration on driver hub
        blueLift = hardwareMap.dcMotor.get("blueLift");
        blackLift = hardwareMap.dcMotor.get("blackLift");

        // Set mode and reset encoder.
        blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse direction of blue lift.
        blueLift.setDirection(DcMotorSimple.Direction.REVERSE);

        blueLift.setTargetPosition(0);
        blackLift.setTargetPosition(0);

        blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        blueLift.setTargetPosition(0);
        blackLift.setTargetPosition(0);
    }




    public DcMotor blueLift;
    public DcMotor blackLift;
    public Servo blackExtend;
    public Servo blueExtend;
    public Servo blackGrip;
    public Servo blueGrip;

    // ImuWrapper is implemented for IMU or Pinpoint.
    public ImuPositionI imuPos;


    @Override
    public void updateImuPos() {
        imuPos.update();
    }

    @Override
    public void resetImu() {
        imuPos.reset();
    }

    @Override
    public void resetImuHeading() {
        imuPos.resetHeading();
    }

    @Override
    public Pose2D getImuPose()  {
        try {
            return imuPos.getPose();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public double getImuHeading(AngleUnit unit) {
        return imuPos.getHeading(unit);
    }

    @Override
    public void setFrontLeftPower(double power) {
        frontLeft.setPower(power);
    }

    @Override
    public void setFrontRightPower(double power) {
        frontRight.setPower(power);

    }

    @Override
    public void setBackLeftPower(double power) {
        backLeft.setPower(power);

    }

    @Override
    public void setBackRightPower(double power) {
        backRight.setPower(power);
    }

    @Override
    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    @Override
    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    @Override
    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    @Override
    public double getBackRightPower() {
        return backRight.getPower();
    }

    @Override
    public void updateState(TimeSourceI timeSource) {
        // No needed when running robot.
        // Maybe could be useful in the future?
    }

    @Override
    public void stopMotors() {
        setFrontLeftPower(0);
        setFrontRightPower(0);
        setBackLeftPower(0);
        setBackRightPower(0);
    }
}