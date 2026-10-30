


package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;

@TeleOp(name = "IMU Test TeleOp", group = "Test")
public class Encoder extends LinearOpMode {
    private DriveTrain driveTrain;
    private PinpointImpl imu;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        imu.reset();

        waitForStart();
        while (opModeIsActive()) {
            imu.update();
            Pose2D pose = imu.getPose();
            double powerScale = 1.0;
            if (gamepad1.x) {
                driveTrain.setFrontLeftPower(powerScale);
            } else {
                driveTrain.setFrontLeftPower(0);

            } if (gamepad1.y) {
                driveTrain.setFrontRightPower(powerScale);
            } else {
                driveTrain.setFrontRightPower(0);

            } if (gamepad1.b) {
                driveTrain.setBackRightPower(powerScale);
            } else {
                driveTrain.setBackRightPower(0);

            } if (gamepad1.a) {
                driveTrain.setBackLeftPower(powerScale);

            } else {
                driveTrain.setBackLeftPower(0);

            }
            dashboardTelemetry.addData("Pose",
                    String.format("HeadingDegrees: %.3f | PoseX: %.3f | PoseY: %.3f",
                            pose.getHeading(AngleUnit.DEGREES),
                            pose.getX(DistanceUnit.INCH),
                            pose.getY(DistanceUnit.INCH)));
            dashboardTelemetry.update();
        }
    }
}
