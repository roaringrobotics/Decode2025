package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import  com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final DcMotorEx shooterMotor;
    public final CRServo blueServo;
   public final CRServo blackServo;
   private double targetVel = 0;;
   public static final double P= 16;
    public static final double I= 0.3;
    public static final double D= 2.5;
    public static final double F= 14;

    //Blocker tuning :)
    private static final double allowedError =50;
    private static final long settleTimeMs = 90;
    private long stableSince = 0;




    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Initialize servos (use hardware names configured in your robot config)
        blueServo = hardwareMap.get(CRServo.class, "blueServo");
        blackServo = hardwareMap.get(CRServo.class, "blackServo");

         //Reverse bottom motor so both spin the same physical direction
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default to BRAKE when power is zero
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Default run mode (change as needed)
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P,I,D,F)
        );

    }

    public void setTargetVelocity(double vel){
        targetVel = vel;
        shooterMotor.setVelocity(vel);
    }
    public boolean isReady(){
        double current = shooterMotor.getVelocity();

        if(Math.abs(current - targetVel) < allowedError){
            if(stableSince ==0)
                stableSince = System.currentTimeMillis();
            return System.currentTimeMillis() - stableSince > settleTimeMs;
        } else{
            stableSince = 0;
            return false;
        }
    }

    public double getVelocity(){
        return shooterMotor.getVelocity();
    }
     public void stop(){
        setTargetVelocity(0);
     }
   /* public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void startShooterMotor(double power) {
        setPower(power);

    }
    public void stopShooterMotor() {
        setPower(0.0);
    }*/

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
        double power = 1.0;
        blueServo.setPower(-power);
        blackServo.setPower(power);
     }
    //public void toggleShootMotor() {
        //if (shooterMotor.getPower() == 0.0) {
          //  startShooterMotor();
        //} else {
          //  stopShooterMotor();
        //}
    }
