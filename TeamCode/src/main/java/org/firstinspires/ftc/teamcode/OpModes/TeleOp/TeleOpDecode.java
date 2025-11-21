package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

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

    // private Intake intake;
    private ButtonState lastAButtonState = ButtonState.NOT_PRESSED;

    private enum ButtonState {
        PRESSED,
        NOT_PRESSED
    }


    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        //  intake = new Intake(hardwareMap);
        waitForStart();
        float deadZone = 0.75F;
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.driveFieldCentric(drive, strafe, rotate, 0.5, hw);


            //if (gamepad2.left_trigger > 0.5) {
                //shooter.startShooterMotor();
            //}else if (gamepad2.left_trigger < 0.5) {
                //shooter.stopShooterMotor();
            //}

            // if last time is was up and this time it's down toggle
            // if last time it was down and this time it's up toggle

            ButtonState currentAButtonState = gamepad2.a ? ButtonState.PRESSED : ButtonState.NOT_PRESSED;
            if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.startShoot();
            } if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.NOT_PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.stopShoot();
            }


            if (gamepad2.dpad_right) {
                shooter.blueServo.setPower(-1);
                shooter.blackServo.setPower(1);
            } else {
                shooter.blueServo.setPower(0);
                shooter.blackServo.setPower(0);
            } if (gamepad2.dpad_left){
                shooter.blueServo.setPower(1);
                shooter.blackServo.setPower(-1);
            }
            if (gamepad1.options) {
                hw.resetImu();
            }


                if (gamepad2.right_trigger > 0.5) {
                    intake.startIntake();
                } else {
                    intake.stopIntake();
                }
                if (gamepad2.left_trigger > 0.5){
                    intake.reverseIntake();
                }

            }


            // Class to hold field centric power level output from getFieldCentricPowerLevels

        }
    }

