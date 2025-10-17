package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.LinearOpModeImpl;
import org.firstinspires.ftc.teamcode.Implementations.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.RobotHardware.Hardware;


@Autonomous
public class RonAuto extends LinearOpMode {
    //Hardware an variables
    Hardware hw;

    ElapsedTime run = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //All movement happen
        Hardware hw = new Hardware(hardwareMap);

        hw.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Parking break
        hw.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime run = new ElapsedTime();
        double drivePower = 0.60;
        double straffPower = 0.50;

        waitForStart();
        run.reset();

        while (run.milliseconds() < 700 && !isStopRequested()) {
            hw.frontLeft.setPower(drivePower);
            hw.frontRight.setPower(drivePower);
            hw.backLeft.setPower(drivePower);
            hw.backRight.setPower(drivePower);
        }
        run.reset();
        while (run.milliseconds() < 700 && !isStopRequested()) {
            hw.frontLeft.setPower(-straffPower);
            hw.frontRight.setPower(straffPower);
            hw.backLeft.setPower(straffPower);
            hw.backRight.setPower(-straffPower);
        }
        run.reset();
    }



}