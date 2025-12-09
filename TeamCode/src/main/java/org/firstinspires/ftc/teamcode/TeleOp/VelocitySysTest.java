package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.VelocityTestHardware;

@TeleOp
public class VelocitySysTest extends OpMode {

    private VelocityTestHardware shooter;

    private static final double SHOOT_SPEED = 1300;

    @Override
    public void init() {
        shooter = new VelocityTestHardware(hardwareMap);

        telemetry.addLine("Velocity Shooter Test Ready");
        telemetry.addLine("RT = Spin up shooter");
        telemetry.addLine("A = Feed ball (only fires when ready)");
        telemetry.update();
    }

    @Override
    public void loop() {


        if (gamepad1.right_trigger > 0.2) {
            shooter.setTargetVelocity(SHOOT_SPEED);
        } else {
            shooter.stop();
            shooter.stopFeed();
        }


        if (gamepad1.a && shooter.isReady()) {
            shooter.feed();
        } else {
            shooter.stopFeed();
        }


        telemetry.addData("Target Velocity", SHOOT_SPEED);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.addData("Ready To Fire", shooter.isReady());
        telemetry.update();
    }
}
