package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.LinearOpModeImpl;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.Implementations.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;
import org.firstinspires.ftc.teamcode.Pathing.PathController;
import org.firstinspires.ftc.teamcode.Pathing.PidParamCollection;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;
import org.firstinspires.ftc.teamcode.TelemetryHelper;\[]
@Autonomous
public class ExampleNewPathing extends LinearOpMode {

    // This runs as soon as init it pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);
        hw.imuPos.reset();
        // wait for start to be pushed.
        waitForStart();

        LinearOpModeImpl ourOp = new LinearOpModeImpl(this);

        PathController pathing = new PathController(hw, ourOp, 0.2,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new SystemTimeSource(), new AndroidLog());

        //pause();
        // Move forward 24 inches.
        //pathing.setTargetHeadingDeg(0);
        pathing.setTargetLocation(12,0);
        pathing.run(20);

//        pause();
//        pathing.setTargetHeadingDeg(45);
//        pathing.setTargetLocation(24,24);
//        pathing.run();
//
//        pause();
//        pathing.setTargetHeadingDeg(-90);
//        pathing.setTargetLocation(36,24);
//        pathing.run();
    }

    private void pause() {
        //TelemetryHelper.UpdateTelemetry(myTelem, "Push B to continue...");
        //while(!gamepad1.b)
            //sleep(250);
    }

    Hardware hw;
    double power = 0.5;
    TelemetryI myTelem = new TelemetryWrapper(this.telemetry);
}
