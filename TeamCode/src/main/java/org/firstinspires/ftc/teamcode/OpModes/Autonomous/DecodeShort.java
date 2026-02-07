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


@Autonomous(name = "Shoot From Triangle", group = "Autonomous")
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
        int rows = 0;
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
            if (gamepad2.dpadUpWasPressed()) {
                rows = (rows + 1) % 4;
            } else if (gamepad2.dpadDownWasPressed()) {
                rows = (rows - 1) % 4;
            }
            telemetry.addData("Rows", rows);
            idle();
        }

        if (isStopRequested()) {
            return;
        }
        double kp = 0.045;
        double ki = 0.0;
        double kd = 0.13;
        // Start autonomous

        // these are for strafing
        double kps = 0.13;
        double kds = 0.29;

        // Start autonomous
        try {
            driveTrain.driveStraight(-6, .5, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(23 * mirrorField, imu, log);
            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                shooter.continousShoot(false, false, true);
            }
            for (int i = 0; i < rows; i++) {
                shooter.resetShooter();
                driveTrain.rotateRelative(-113 * mirrorField, imu, log);
                driveTrain.driveStrafe(-20 - i * 20, .8, imu, log, kps, ki, kds);
                shooter.startIntake();
                driveTrain.driveStraight(40, .5, imu, log, kp, ki, kd);
                shooter.stopIntake();
                driveTrain.driveStraight(-40, .8, imu, log, kp, ki, kd);
                driveTrain.rotateRelative(90 * mirrorField, imu, log);
                driveTrain.driveStraight(15 + i * 20, .8, imu, log, kps, ki, kds);
                driveTrain.rotateRelative(23 * mirrorField, imu, log);
                stateTimer.reset();
                shooter.resetShooter();
                while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                    shooter.continousShoot(false, false, true);
                }
                if (i == 1){
                    while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                        shooter.continousShoot(false, true, false);
                }
                }
                if(i >= 2) {
                    while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                        shooter.continousShoot(false, true, false);
                    }
                }
            }
    }   catch (Exception e) {
            throw new RuntimeException(e);
        }












        }
    }









