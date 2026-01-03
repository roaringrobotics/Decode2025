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
            
            telemetry.addData("Side", team);
            idle();
        }

        if (isStopRequested()) {
            return;
        }
        // Start autonomous
        stateTimer.reset();
        sleep(5000);
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
        while (Math.abs(x) < targetDistance && opModeIsActive()) {
            double PIDpower = pid.calculate(targetDistance, x);
            PIDpower = Util.clamp(PIDpower, -1, 1);
            log.d("", "===========================================");
            log.d("Power1", String.valueOf(PIDpower));
            PIDpower = rampDrive.getValue(PIDpower);
            log.d("Power2", String.valueOf(PIDpower));

            driveTrain.setFrontLeftPower(PIDpower);
            driveTrain.setFrontRightPower(PIDpower);
            driveTrain.setBackLeftPower(PIDpower);
            driveTrain.setBackRightPower(PIDpower);

           // driveTrain.followHeading(imu, startHeading, PIDpower, 0.03, 2);

            // Update y position from hardware (placeholder logic)
            imu.update();
            y = imu.getPosY();
            x = imu.getPosX();
            h = imu.getHeading(AngleUnit.DEGREES);

            telemetry.update();
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
            driveTrain.rotate(90, 0.4, imu, log);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        try {
            driveTrain.driveStraight(24, -0.5, imu, log);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
