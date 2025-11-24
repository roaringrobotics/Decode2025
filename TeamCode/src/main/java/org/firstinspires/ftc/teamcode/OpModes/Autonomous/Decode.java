// language: java
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Interfaces.LogI;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpDecode;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
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
        double target = 0;
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
        target = -15;
        telemetry.clearAll();
        telemetry.addData("Status", "Started");
        telemetry.addData("X Position", x);
        telemetry.addData("Y Position", y);
        telemetry.addData("Target", target);
        while (x > target && opModeIsActive()) {
            // Drive forward at half power
            driveTrain.setFrontLeftPower(0.5);
            driveTrain.setFrontRightPower(0.5);
            driveTrain.setBackLeftPower(0.5);
            driveTrain.setBackRightPower(0.5);

            // Update y position from hardware (placeholder logic)
            imu.update();
            y = imu.getPosY();
            x = imu.getPosX();

            telemetry.update();
            log.d("", "===========================================");
            log.d("X Position: ", String.valueOf(x));
            log.d("Y Position: ", String.valueOf(y));
            log.d("Target: ", String.valueOf(target));
            log.d("", "===========================================");
        }
        log.d("Target ", "Target Reached, Stopping");
        log.d("", "===========================================");
        driveTrain.setFrontLeftPower(0);
        driveTrain.setFrontRightPower(0);
        driveTrain.setBackLeftPower(0);
        driveTrain.setBackRightPower(0);
        imu.update();
        y = imu.getPosY();
        x = imu.getPosX();
        telemetry.update();
        log.d("", "End Positions:");
        log.d("X Position: ", String.valueOf(x));
        log.d("Y Position: ", String.valueOf(y));
        log.d("Target: ", String.valueOf(target));

    }
}
