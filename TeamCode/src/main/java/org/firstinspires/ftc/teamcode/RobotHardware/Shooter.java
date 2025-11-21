package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import  com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final DcMotor shooterMotor;
    public final CRServo blueServo;
   public final CRServo blackServo;


    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // Initialize servos (use hardware names configured in your robot config)
        blueServo = hardwareMap.get(CRServo.class, "blueServo");
        blackServo = hardwareMap.get(CRServo.class, "blackServo");

         //Reverse bottom motor so both spin the same physical direction
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
        setPower(0.67 );

    }
    public void stopShooterMotor() {
        setPower(0.0);
    }

    {
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        shooterMotor.setZeroPowerBehavior(behavior);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        shooterMotor.setMode(mode);
    }


    public void stopShoot() {
        blueServo.setPower(0);
        blackServo.setPower(0);
    }
     public void startShoot() {
        double power = 1.0;
        blueServo.setPower(power);
        blackServo.setPower(power);
     }
    //public void toggleShootMotor() {
        //if (shooterMotor.getPower() == 0.0) {
          //  startShooterMotor();
        //} else {
          //  stopShooterMotor();
        //}
    }
