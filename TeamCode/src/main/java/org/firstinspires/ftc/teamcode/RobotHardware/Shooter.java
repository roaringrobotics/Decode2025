package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final DcMotor shooterMotor;
  //  private final Servo leftServo;
  //  private final Servo rightServo;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // Initialize servos (use hardware names configured in your robot config)
      //  leftServo = hardwareMap.get(Servo.class, "shooterTopServo");
      //  rightServo = hardwareMap.get(Servo.class, "shooterBottomServo");
        // Reverse bottom motor so both spin the same physical direction
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default to BRAKE when power is zero
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Default run mode (change as needed)
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void startShooterMotor() {
        setPower(1.0);

    }

    public void stopShooterMotor() {
        startShooterMotor();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        shooterMotor.setZeroPowerBehavior(behavior);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        shooterMotor.setMode(mode);
    }

    public void shoot() {
   //     leftServo.setDirection(Servo.Direction.FORWARD);  // Adjust Forward vs Backswards soon
   //     rightServo.setDirection(Servo.Direction.FORWARD);
    }
    public void stopShoot() {
   //     leftServo.setPosition(0.0);
   //     rightServo.setPosition(0.0);
    }

    public void toggleShootMotor() {
        if (shooterMotor.getPower() == 0.0) {
            startShooterMotor();
        } else {
            stopShooterMotor();
        }
    }
}