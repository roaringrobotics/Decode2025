


package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class Encoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Find a motor in the hardware map named "Arm Motor"
        DcMotor motorR = hardwareMap.dcMotor.get("frontRight");
        // Reset the motor encoder so that it reads zero ticks
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor motorL = hardwareMap.dcMotor.get("backLeft");
        // Reset the motor encoder so that it reads zero ticks
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            // Get the current position of the motor
            int positionr = motorR.getCurrentPosition();
            // Get the current position of the motor
            int positionl = motorL.getCurrentPosition();

            // Show the position of the motor on telemetry
            telemetry.addData("Encoder Position(no (Left)", positionl);
            // Show the position of the motor on telemetry
            telemetry.addData("Encoder Position(no (right)", positionr);
            telemetry.update();
        }


        while (opModeIsActive()) {

        }
    }
}
