package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;

@TeleOp(group = "Test")
public class MotorTest extends LinearOpMode {
    private DriveTrain driveTrain;

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                driveTrain.setFrontLeftPower(0.25);
            } else if (gamepad1.y) {
                driveTrain.setFrontRightPower(0.25);
            } else if (gamepad1.b) {
                driveTrain.setBackRightPower(0.25);
            } else if (gamepad1.a) {
                driveTrain.setBackLeftPower(0.25);
            } else {
                driveTrain.setFrontLeftPower(0);
                driveTrain.setFrontRightPower(0);
                driveTrain.setBackRightPower(0);
                driveTrain.setBackLeftPower(0);
            }
        }
    }

}