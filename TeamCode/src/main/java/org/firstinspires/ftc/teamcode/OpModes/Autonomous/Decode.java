// language: java
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpDecode;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
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
    private Hardware hw;
    private Intake intake;

    // Hardware placeholders - change names to match your robot configuration
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


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


    }
}
