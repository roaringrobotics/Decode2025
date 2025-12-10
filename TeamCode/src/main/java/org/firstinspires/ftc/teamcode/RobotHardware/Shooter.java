package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final DcMotorEx shooterMotor;
    public final CRServo blueServo;
    public final CRServo blackServo;
    public long lastTime;
    public double lastVelocity;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Initialize servos (use hardware names configured in your robot config)
        blueServo = hardwareMap.get(CRServo.class, "blueServo");
        blackServo = hardwareMap.get(CRServo.class, "blackServo");

         //Reverse bottom motor so both spin the same physical direction
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default to BRAKE when power is zero
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void updateKinematics(long currentTime)
    {
        lastTime = currentTime;
        lastVelocity = shooterMotor.getVelocity();
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void startShooterMotor(double power) {
        setPower(0.5);

    }
    public void stopShooterMotor() {
        setPower(0.0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        shooterMotor.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        shooterMotor.setMode(mode);
    }


    public void stopShoot() {
        blueServo.setPower(0);
        blackServo.setPower(0);
    }
     public void startShoot() {
        blueServo.setPower(-1);
        blackServo.setPower(1);
     }

    public void toggleShooterMotor(boolean toggle) {
        if(toggle)
        {
          if(shooterMotor.getPower() != 0 )  // motor running
              stopShooterMotor();
          else
              startShooterMotor(0.5);
        }
    }

    public void toggleShooterFeeders(boolean toggle) {
        if(toggle)
        {
            if(blueServo.getPower() != 0 ) // feeders running
                stopShoot();
            else
                startShoot();
        }
    }
    //public void toggleShootMotor() {
        //if (shooterMotor.getPower() == 0.0) {
          //  startShooterMotor();
        //} else {
          //  stopShooterMotor();
        //}
    }
