package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
import org.firstinspires.ftc.teamcode.Implementations.SystemTimeSource;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;

import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;
import org.firstinspires.ftc.teamcode.RobotHardware.VelocityTestHardware;

@Autonomous
public class FreeThrow extends LinearOpMode {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Hardware hw;
    private Intake intake;
    private VelocitySysTest Test;
    private VelocityTestHardware Shooter;
    private SystemTimeSource timesource = new SystemTimeSource();
    private static final double SHOOT_SPEEDShort = 3300;
    private static final double SHOOT_SPEEDLong = 3900;
    private static final double P = 20;
    private static final double I = 0.3;
    private static final double D = 2.5;
    private static final double F = 10.2;

    // Velocity blocker settings
    private static final double allowedError = 50;  // TPS
    private static final long settleTimeMs = 90;    // ms
    private long stableSince = 0;
    private AndroidLog log = new AndroidLog();

    // private Intake intake;
    private ButtonState lastAButtonState = ButtonState.NOT_PRESSED;

    private enum ButtonState {
        PRESSED,
        NOT_PRESSED
    }

    // 3. Add a timer to debounce the button
    private ElapsedTime buttonTimer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        hw = new Hardware(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake, log);
        Shooter = new VelocityTestHardware(hardwareMap);
        //  intake = new Intake(hardwareMap);
        waitForStart();
        shooter.startShooterMotor(1);
        sleep(5000);
        shooter.startShoot();
        intake.startIntake(1.0);
    }
}

