package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;

@TeleOp
public class TeleOpDecode extends LinearOpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Hardware hw;
    private Intake intake;
    private SystemTimeSource timesource = new SystemTimeSource();
    private final AndroidLog log = new AndroidLog();

    // private Intake intake;
    private ButtonState lastAButtonState = ButtonState.NOT_PRESSED;

    private enum ButtonState {
        PRESSED,
        NOT_PRESSED
    }

    // 3. Add a timer to debounce the button
    private ElapsedTime buttonTimer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);
        //  intake = new Intake(hardwareMap);
        waitForStart();
        float deadZone = 0.75F;
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.driveFieldCentric(drive, strafe, rotate, 1, hw);

            timesource.update();

            shooter.continousShoot(gamepad2.a, gamepad2.b, gamepad2.y);

            if(!gamepad2.a && !gamepad2.b && !gamepad2.y) {

                if (gamepad2.right_trigger > 0.5) {
                    shooter.startIntake();
                } else {
                    shooter.stopIntake();
                }
                if (gamepad2.left_trigger > 0.5) {
                    shooter.reverseIntake();
                }
            }
            if (gamepad1.options) {
                hw.resetImu();
            }

        }
    }
}

