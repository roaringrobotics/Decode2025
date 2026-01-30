package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.VelocityTestHardware;

//@TeleOp
public class VelocitySysTest extends OpMode {

    private VelocityTestHardware shooter;

    private static final double SHOOT_SPEED = 1300;
    private Intake intake;

    @Override
    public void init() {
        shooter = new VelocityTestHardware(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addLine("Velocity Shooter Test Ready");
        telemetry.addLine("RT = Spin up shooter");
        telemetry.addLine("A = Feed ball (only fires when ready)");
        telemetry.update();
    }

    @Override
    public void loop() {


        if (gamepad1.right_bumper) {
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
        if (gamepad1.right_trigger > 0.5) {
            intake.startIntake(1);
        } else {
            intake.stopIntake();
        }
        if (gamepad1.left_trigger > 0.5){
            intake.reverseIntake();
        }


        telemetry.addData("Target Velocity", SHOOT_SPEED);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.addData("Ready To Fire", shooter.isReady());
        telemetry.update();
    }
}
