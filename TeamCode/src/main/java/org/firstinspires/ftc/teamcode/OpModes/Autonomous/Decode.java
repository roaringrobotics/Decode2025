// language: java
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Math.Util;
import org.firstinspires.ftc.teamcode.Pathing.PID;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;


/*
  Autonomous skeleton:
  - simple state machine (INIT -> DRIVE -> TURN -> ACTION -> IDLE -> DONE)
  - hardware placeholders (left/right drive motors)
  - timed transitions using ElapsedTime
  - helper methods to set/stop drive power
*/

@Autonomous(name = "DecodeLong", group = "Autonomous")
public class Decode extends LinearOpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private PinpointImpl imu;
    private Intake intake;
    private final AndroidLog log = new AndroidLog();


    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        imu = new PinpointImpl(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);
        PID pid = new PID(0.85, 0, 0);
        PID pidRotate = new PID(0.01, 0, 0);
        PowerRampController rampDrive = new PowerRampController(
                .2,
                new SystemTimeSource());


//        Through testing y's and x's are flipped
//        instead of:
//                y
//                |
//                |
//       ------------------x
//                |
//                |
//         we have:
//                x
//                |
//                |
//       ------------------y
//                |
//                |

        double y = 0;
        double x = 0;
        double h = 0;
        double targetDistance = 0;
        double mirrorField = 1;
        String team = "Blue Side";
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();

        double kp = 0.045;
        double ki = 0.0;
        double kd = 0.0;

        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            if (gamepad2.optionsWasPressed()) {
                mirrorField = mirrorField * -1;
            }
            if (mirrorField == -1) {
                team = "Red Side";
            } else {
                team = "Blue Side";
            }
            //telemetry.addData("Add Delay, before robot starts?");
            //if(gamepad2.start){
                //sleep(5000);
              //  telemetry.addData("Delay Added")
            //} else {
            //    sleep(0);
            //}

            double change = 0.01;
            if(gamepad2.right_bumper) {
                change = .001;
            }

            if(gamepad2.a && gamepad2.dpad_up) {
                kd += .01;
            }
            else if(gamepad2.a && gamepad2.dpad_down) {
                kd -= .01;
            }
            else if(gamepad2.b && gamepad2.dpad_up) {
                ki += .00001;
            }
            else if(gamepad2.b && gamepad2.dpad_down) {
                ki -= .00001;
            }
            else if(gamepad2.y && gamepad2.dpad_up) {
                kp += .01;
            }
            else if(gamepad2.y && gamepad2.dpad_down) {
                kp -= .01;
            }

            telemetry.addData("PID Values:", "kp: %.3f ki: %.5f kd: %.3f", kp, ki, kd);
            
            telemetry.addData("Side", team);

            sleep(100);
            idle();
        }

        // Start autonomous
        stateTimer.reset();
        //sleep(5000);
        targetDistance = 52;

        telemetry.clearAll();
        telemetry.addData("Status", "Started");
        telemetry.addData("X Position", x);
        telemetry.addData("Y Position", y);
        telemetry.addData("Heading", h);
        telemetry.addData("Target", targetDistance);

        shooter.startShooterMotor(0.7);

        int count = 0;
        double startHeading = imu.getHeading(AngleUnit.DEGREES);

        try {
            driveTrain.driveStraight(78, 0.6, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(45, imu, log);
            while (shooter.getBallsLaunched() < 3 && opModeIsActive()) {
                shooter.continousShoot(true, false, false);
                sleep(5);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        if (!isStopRequested())
            return;
        log.d("Target", "Target Reached, Turning");
        log.d("", "===========================================");
        driveTrain.setFrontLeftPower(0);
        driveTrain.setFrontRightPower(0);
        driveTrain.setBackLeftPower(0);
        driveTrain.setBackRightPower(0);

        telemetry.update();
        y = imu.getPosY();
        x = imu.getPosX();
        log.d("X Position", String.valueOf(x));
        log.d("Y Position", String.valueOf(y));
        log.d("Heading", String.valueOf(h));
        log.d("Target", String.valueOf(targetDistance));

        // Rotate
        count = 0;

        double targetHeading = 38;
        rampDrive.Reset();
        while (Math.abs(h) < Math.abs(targetHeading) && opModeIsActive()) {
            double PIDpower = pidRotate.calculate(targetHeading, h);
            PIDpower = Util.clamp(PIDpower, -1, 1);
//            PIDpower = rampDrive.getValue(PIDpower);
            driveTrain.setFrontLeftPower(-PIDpower * mirrorField);
            driveTrain.setFrontRightPower(PIDpower * mirrorField);
            driveTrain.setBackLeftPower(-PIDpower * mirrorField);
            driveTrain.setBackRightPower(PIDpower * mirrorField);

            imu.update();
            h = imu.getHeading(AngleUnit.DEGREES) * mirrorField;
            telemetry.update();
            if (count > 200) {
                log.d("", "===========================================");
                log.d("Heading ", String.valueOf(h));
                log.d("Target", String.valueOf(targetHeading));
                log.d("", "===========================================");
                count = 0;
            } else {
                count++;
            }
        }
        log.d("Target", "Target Reached Heading, Stopping");
        log.d("", "===========================================");
        driveTrain.setFrontLeftPower(0);
        driveTrain.setFrontRightPower(0);
        driveTrain.setBackLeftPower(0);
        driveTrain.setBackRightPower(0);


        telemetry.update();
        y = imu.getPosY();
        x = imu.getPosX();
        log.d("", "End Positions for Shooting:");
        log.d("X Position", String.valueOf(x));
        log.d("Y Position", String.valueOf(y));
        log.d("Heading: ", String.valueOf(h));

        // Shooting
        stateTimer.reset();
        while (shooter.getBallsLaunched() < 3 && stateTimer.milliseconds() < 5000) {
            shooter.continousShoot(true, false, false);
            sleep(5);
        }
        shooter.stopShoot();
        shooter.stopShooterMotor();
        shooter.stopIntake();
        intake.stopIntake();

        try {
            driveTrain.rotateRelative(90, imu, log);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        try {
//            driveTrain.driveStraight(24, -0.5, imu, log);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
