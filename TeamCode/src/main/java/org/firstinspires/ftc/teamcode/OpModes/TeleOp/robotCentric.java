package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;
@TeleOp
public class robotCentric extends LinearOpMode {
    private DriveTrain driveTrain;

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double powerScale = driveTrain.powerScaler(gamepad1.right_bumper, gamepad1.left_bumper);
            double fieldX = -gamepad1.left_stick_y;
            double fieldY = -gamepad1.left_stick_x; // the negative is a hotfix
            double rotate = gamepad1.right_stick_x;

            telemetry.addData("Power Scale", powerScale);
            telemetry.update();
            driveTrain.driveRobotCentric(fieldX, fieldY, rotate, powerScale);
        }
    }

}
