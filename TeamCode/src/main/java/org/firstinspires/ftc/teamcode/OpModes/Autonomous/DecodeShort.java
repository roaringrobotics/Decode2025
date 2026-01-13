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

// add back in this line if you want to use this opmode
//@Autonomous(name = "DecodeShort", group = "Autonomous")
public class DecodeShort extends LinearOpMode {

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
        PID pid = new PID(0.5, 0, 0);
        PID pidRotate = new PID(0.02, 0, 0);
        PowerRampController rampDrive = new PowerRampController(
                .1,
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
            telemetry.addData("Side", team);
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        // Start autonomous
        stateTimer.reset();
        sleep(5000);
        targetDistance = 4;

        shooter.startShooterMotor(0.85);

        int count = 0;
        while (Math.abs(x) < targetDistance && opModeIsActive()) {
            double PIDpower = pid.calculate(targetDistance, x);
            PIDpower = Util.clamp(PIDpower, -1, 1);
            PIDpower = rampDrive.getValue(PIDpower);

            driveTrain.setFrontLeftPower(PIDpower);
            driveTrain.setFrontRightPower(PIDpower);
            driveTrain.setBackLeftPower(PIDpower);
            driveTrain.setBackRightPower(PIDpower);

            // Update y position from hardware (placeholder logic)
            imu.update();
            y = imu.getPosY();
            x = imu.getPosX();
            h = imu.getHeading(AngleUnit.DEGREES);

            telemetry.update();
            if (10 < count && count < 20) {
                log.d("", "===========================================");
                log.d("Power 1", String.valueOf(PIDpower));
                PIDpower = rampDrive.getValue(PIDpower);
                log.d("Power 2", String.valueOf(PIDpower));

            }
            if (count > 200) {
                log.d("", "===========================================");
                log.d("X Position", String.valueOf(x));
                log.d("Y Position", String.valueOf(y));
                log.d("Heading", String.valueOf(h));
                log.d("Target", String.valueOf(targetDistance));
                log.d("", "===========================================");
                count = 0;
            } else {
                count++;
            }
        }
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
        double targetHeading = 22;
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
            h = imu.getHeading(AngleUnit.DEGREES);
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

            sleep(3000);
            shooter.startShooterMotor(0.88);
            shooter.startShootServos();
            intake.startIntake(0.8);
            sleep(5000);


            shooter.stopShootServos();
            shooter.stopShooterMotor();
            intake.stopIntake();
        }
    }


