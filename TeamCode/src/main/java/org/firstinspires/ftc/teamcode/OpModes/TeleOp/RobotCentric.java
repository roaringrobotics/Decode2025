package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Robot Centric Drive", group = "Test")
public class RobotCentric extends LinearOpMode {
    private DriveTrain driveTrain;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double powerScale = driveTrain.powerScaler(gamepad1.right_bumper, gamepad1.left_bumper);
            double fieldX = -gamepad1.left_stick_x;
            double fieldY = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;


            driveTrain.driveRobotCentric(fieldX, fieldY, rotate, powerScale);
        }
    }

}
