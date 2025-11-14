package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private ButtonState lastAButtonState = ButtonState.NOT_PRESSED;
    private enum ButtonState {
        PRESSED,
        NOT_PRESSED
    }

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        waitForStart();
        float deadZone = 0.75F;
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.driveFieldCentric(drive, strafe, rotate, 0.5, hw);
            if (gamepad2.left_trigger > deadZone) {
                shooter.startShooterMotor();
            }

            // if last time is was up and this time it's down toggle
            // if last time it was down and this time it's up toggle

            ButtonState currentAButtonState = gamepad2.a ? ButtonState.PRESSED : ButtonState.NOT_PRESSED;
            if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.NOT_PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.toggleShootMotor();
            }
            if (gamepad2.right_trigger > deadZone) {
                shooter.shoot();
            }
            if (gamepad2.b) {
                intake.startIntake();
            }
            else{
                intake.stopIntake();
            }

        }

        // Class to hold field centric power level output from getFieldCentricPowerLevels

    }
}
