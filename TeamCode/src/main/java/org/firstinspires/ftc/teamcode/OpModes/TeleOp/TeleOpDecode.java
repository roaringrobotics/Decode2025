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
        //  intake = new Intake(hardwareMap);
        waitForStart();
        float deadZone = 0.75F;
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.driveFieldCentric(drive, strafe, rotate, 1, hw);


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
                shooter.startShooterMotor(0.5);
            } if (currentAButtonState != lastAButtonState && currentAButtonState == ButtonState.NOT_PRESSED) {
                lastAButtonState = currentAButtonState;
                shooter.stopShooterMotor();
            }

            if (gamepad2.aWasPressed()){
                if(motorRunning){
                    shooter.stopShoot();
                    motorRunning = true;
                } else {
                    shooter.startShoot();
                    motorRunning = false;
                }
            }
            if (gamepad2.b && buttonTimer.seconds() > 0.3) {
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
            } if (gamepad2.dpad_left){
                shooter.blueServo.setPower(1);
                shooter.blackServo.setPower(-1);
            }
            if (gamepad1.options) {
                hw.resetImu();
            }


                if (gamepad2.right_trigger > 0.5) {
                    intake.startIntake(1);
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

