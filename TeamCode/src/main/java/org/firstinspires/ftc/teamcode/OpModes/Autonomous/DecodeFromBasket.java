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

@Autonomous(name = "DecodeFromBasket", group = "Autonomous")
public class DecodeFromBasket extends LinearOpMode {

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
        PID pid = new PID(0.8, 0, 0);
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
        double mirrorField = 1;
        String team;
        double targetDistance;
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
        try {
            // Please sky, rain
            shooter.startShooterMotor(0.6);
            driveTrain.driveStraight(-55, 0.8, imu, log, kp, ki, kd);

            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 5000) {
                shooter.continousShoot(false, true, false);
                sleep(5);
            }
            shooter.resetShooter();
            sleep(1);
            driveTrain.driveStrafe(-13, 0.8, imu, log, kps, ki, kds);
            driveTrain.rotateRelative(-142 * mirrorField, imu, log);
            shooter.startIntake();
            driveTrain.driveStraight(-38, 0.4, imu, log, kp, ki, kd);
            shooter.resetShooter();
            sleep(1);
            driveTrain.driveStraight(38, 0.8, imu, log, kp, ki, kd);
            driveTrain.rotateRelative(135 * mirrorField, imu, log);

            stateTimer.reset();
            while (shooter.getBallsLaunched() < 3 && opModeIsActive() && stateTimer.milliseconds() < 4000) {
                shooter.continousShoot(false, true, false);
                sleep(5);
            }
            shooter.resetShooter();
            driveTrain.rotateRelative(-135 * mirrorField, imu, log);
            driveTrain.driveStrafe(24, 0.7, imu, log, kps, ki, kds);
            shooter.startIntake();
            driveTrain.driveStraight(-40, 0.6, imu, log, kp, ki, kd);
            shooter.resetShooter();
            driveTrain.driveStraight(4, 0.6, imu, log, kp, ki, kd);
//
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}