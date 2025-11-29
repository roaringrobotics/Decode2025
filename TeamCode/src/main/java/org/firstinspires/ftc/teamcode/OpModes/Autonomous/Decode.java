// language: java
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Pathing.PID;
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

@Autonomous(name = "Decode", group = "Autonomous")
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
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        PID pid = new PID(0.1,0,0);

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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu.reset();


        // show init telemetry until start pressed
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        // Start autonomous
        stateTimer.reset();
        targetDistance = 48;

        telemetry.clearAll();
        telemetry.addData("Status", "Started");
        telemetry.addData("X Position", x);
        telemetry.addData("Y Position", y);
        telemetry.addData("Heading", h);
        telemetry.addData("Target", targetDistance);
        int count = 0;
        while (Math.abs(x) < targetDistance && opModeIsActive()) {
            double PIDpower = pid.calculate(targetDistance, x);
            driveTrain.setFrontLeftPower(0.5 * PIDpower);
            driveTrain.setFrontRightPower(0.5 * PIDpower);
            driveTrain.setBackLeftPower(0.5 * PIDpower);
            driveTrain.setBackRightPower(0.5 * PIDpower);

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
        double targetHeading = 45;
        while (h < targetHeading && opModeIsActive()) {
            double PIDpower = pid.calculate(targetHeading, h);
            driveTrain.setFrontLeftPower(-0.5 * PIDpower);
            driveTrain.setFrontRightPower(0.5 * PIDpower);
            driveTrain.setBackLeftPower(-0.5 * PIDpower);
            driveTrain.setBackRightPower(0.5 * PIDpower);

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
            log.d("", "End Positions:");
            log.d("X Position", String.valueOf(x));
            log.d("Y Position", String.valueOf(y));
            log.d("Heading: ", String.valueOf(h));


            shooter.startShooterMotor(0.5);
            sleep(4000);
            shooter.startShoot();
            intake.startIntake();
            sleep(5000);



    }
}
