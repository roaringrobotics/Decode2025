package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Interfaces.ImuPositionI;

// Implement an ImuWrapper for internal IMU and encoders.
public class ImuImpl implements ImuPositionI {


    public ImuImpl(HardwareMap hwmap)
    {
        imu = hwmap.get(IMU.class, "imu");
        yEncoder = hwmap.dcMotor.get("TBD");
        xEncoder = hwmap.dcMotor.get("TBD");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    @Override
    public void reset() {
        imu.resetYaw();
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void resetHeading() {
        imu.resetYaw();
    }

    @Override
    public double getHeading(AngleUnit units) {
        YawPitchRollAngles angels =  imu.getRobotYawPitchRollAngles();
        if(units == AngleUnit.DEGREES)
            return Math.toDegrees(angels.getYaw());
        else
            return angels.getYaw();
    }

    @Override
    public double getPosY() {
        return yEncoder.getCurrentPosition() * ticsPerInch;
    }

    @Override
    public double getPosX() {
        return xEncoder.getCurrentPosition() * ticsPerInch;
    }

    @Override
    public void update() {
        // not needed for IMU and encoders.
    }

    @Override
    public Pose2D getPose() throws Exception {
        throw new Exception("not implemented");
    }

    private final IMU imu;
    private final DcMotor yEncoder;
    private final DcMotor xEncoder;
    private final double ticsPerInch = 72;
}
