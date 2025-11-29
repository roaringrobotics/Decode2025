package org.firstinspires.ftc.teamcode.Pathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware.PinpointImpl;

public class PIDTuning extends LinearOpMode {
    DriveTrain driveTrain = new DriveTrain(hardwareMap);
    PinpointImpl imu = new PinpointImpl(hardwareMap);
    PID pid;

    // These are only needed for testing
    double targetDistance = 60;
    double targetHeading = 90;
    double x;
    double h;
    double kP = 0;
    double kD = 0;
    double sensitivity = 0.01;


    @Override
    public void runOpMode() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hint", "Waiting for start - update sensors/vision here");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            return;
        }

        while (!gamepad1.aWasReleased()) {
            if (gamepad1.bWasReleased()) {
                telemetry.addData("Status", "Tuning kD");
                telemetry.addData("Press up and down on the D-pad to increase or decrease kD, kD:", kD);
                telemetry.addData("Left and right changes sensitivity, sensitivity: ", sensitivity);
                telemetry.addData("Press b to switch to Tuning kD", "");
                telemetry.addData("Press a to start", "");
                if (gamepad1.dpadUpWasReleased()) {
                    kD += sensitivity;
                }
                // Continue Here
            } else {
                telemetry.addData("Status", "Tuning kP");
                telemetry.addData("Press up and down on the D-pad to increase or decrease kP, kP:", kP);
                telemetry.addData("Left and right changes sensitivity, sensitivity: ", kP);
                telemetry.addData("Press b to switch to Tuning kD", "");
                telemetry.addData("Press a to start", "");

            }
        }

            x = imu.getPosX();
            while (Math.abs(x) < targetDistance && opModeIsActive()) {

                x = imu.getPosX();
                double PIDpower = pid.calculate(targetDistance, x);
                driveTrain.setFrontLeftPower(0.5 * PIDpower);
                driveTrain.setFrontRightPower(0.5 * PIDpower);
                driveTrain.setBackLeftPower(0.5 * PIDpower);
                driveTrain.setBackRightPower(0.5 * PIDpower);
            }
            while (Math.abs(h) < targetHeading && opModeIsActive()) {

                h = imu.getHeading(AngleUnit.DEGREES);
                double PIDpower = pid.calculate(targetDistance, x);
                driveTrain.setFrontLeftPower(-0.5 * PIDpower);
                driveTrain.setFrontRightPower(0.5 * PIDpower);
                driveTrain.setBackLeftPower(-0.5 * PIDpower);
                driveTrain.setBackRightPower(0.5 * PIDpower);
            }
        }
    }

