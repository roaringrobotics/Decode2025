package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.speech.tts.TextToSpeech;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;


//
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

    private boolean motorRunning;

    private enum ShooterState {
        RUNNING,
        STOPPED
    }

    // 2. Set the initial state
    private ShooterState shooterState = ShooterState.STOPPED;

    // 3. Add a timer to debounce the button
    private ElapsedTime buttonTimer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        double power;
        hw.imuPos.resetHeading();
        //  intake = new Intake(hardwareMap);
        waitForStart();
        float deadZone = 0.5F;
        while (opModeIsActive()) {

            hw.imuPos.update();

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            power = powerScaler();
            driveTrain.driveFieldCentric(drive, strafe, rotate, power, hw);





            ButtonState currentAButtonState = gamepad2.a ? ButtonState.PRESSED : ButtonState.NOT_PRESSED;
            if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.startShooterMotor(0.7);
            }
            if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.NOT_PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.stopShooterMotor();
            }

            if (gamepad2.aWasPressed()) {
                if (motorRunning) {
                    shooter.stopShoot();
                    motorRunning = true;
                } else {
                    shooter.startShoot();
                    motorRunning = false;
                }
            }
            if (gamepad2.bWasReleased()) {
                buttonTimer.reset(); // Reset timer to prevent rapid toggling
                switch (shooterState) {
                    case STOPPED:
                        shooter.startShooterMotor(0.7);
                        shooterState = ShooterState.RUNNING;
                        break;
                    case RUNNING:
                        shooter.stopShooterMotor();
                        shooterState = ShooterState.STOPPED;
                        break;
                }
            }


            if (gamepad2.dpad_right) {
                shooter.blueServo.setPower(-1);
                shooter.blackServo.setPower(1);
            } else {
                shooter.blueServo.setPower(0);
                shooter.blackServo.setPower(0);
            }
            if (gamepad2.dpad_left) {
                shooter.blueServo.setPower(1);
                shooter.blackServo.setPower(-1);
            }

            if (gamepad1.options) {
                hw.resetImu();
            }


            if (gamepad2.right_trigger > deadZone) {
                intake.startIntake(0.75);
            } else {
                intake.stopIntake();
            }
            if (gamepad2.left_trigger > 0.5) {
                intake.reverseIntake();
            }

        }


        // Class to hold field centric power level output from getFieldCentricPowerLevels

    }

    private double powerScaler() {

        if (gamepad1.right_bumper) {
            return 0.5;

        } else if (gamepad1.left_bumper) {
            return 0.25;

        } else {
            return 1.0;
        }
    }
}


