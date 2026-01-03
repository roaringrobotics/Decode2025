package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;

public class Shooter {
    private final DcMotorEx shooterMotor;
    public final CRServo blueServo;
    public final CRServo blackServo;
    public long lastTime;
    public double lastVelocity;
    //private double[] velocityBuffer = {(1430*Math.PI), (1573*Math.PI), (1716*Math.PI)};
    private double[] velocityBuffer = {(1300), (1400), (1500)};
    private double allowedError = 50.0;
    private Intake intake;
    public AndroidLog log;


    int ballsLaunched = 0;
    boolean empty = false;

    enum ShooterState {
        IDLE,
        SPINNING_UP,
        SHOOTING
    }
    private ShooterState ShooterState;
    private ShooterState lastShooterState;

    public Shooter(HardwareMap hardwareMap, Intake in, AndroidLog logIn) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = in;
        ShooterState = ShooterState.IDLE;
        log = logIn;



        // Initialize servos (use hardware names configured in your robot config)
        blueServo = hardwareMap.get(CRServo.class, "blueServo");
        blackServo = hardwareMap.get(CRServo.class, "blackServo");

        //Reverse bottom motor so both spin the same physical direction
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Default to BRAKE when power is zero
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Reset encoder counts first (optional but recommended)
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Switch to RUN_USING_ENCODER mode
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateKinematics(long currentTime) {
        lastTime = currentTime;
        lastVelocity = shooterMotor.getVelocity();
    }


    public void startShooterMotor(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    public void stopShooterMotor() {
        shooterMotor.setVelocity(0.0);
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

    public void toggleShooterMotor65(boolean toggle) {
        if (toggle) {
            if (shooterMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.65);
        }
    }

    public void toggleShooterMotor60(boolean toggle) {
        if (toggle) {
            if (shooterMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.6);
        }
    }

    public void toggleShooterMotor55(boolean toggle) {
        if (toggle) {
            if (shooterMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.55);
        }
    }

    public void toggleShooterMotor50(boolean toggle) {
        if (toggle) {
            if (shooterMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.5);
        }
    }

    public void toggleShooterFeeders(boolean toggle) {
        if (toggle) {
            if (blueServo.getPower() != 0) // feeders running
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
    public void getShooterVelocity() {
        shooterMotor.getVelocity();
    }

    public void continousShoot(boolean buttonShort, boolean buttonMid, boolean buttonLong) {
        boolean allOff = !buttonShort && !buttonMid && !buttonLong;
        boolean anyOn = buttonShort || buttonMid || buttonLong;

        double targetVelocity = 0.0;
        if(buttonShort) {
            targetVelocity = velocityBuffer[0];
        } else if (buttonMid) {
            targetVelocity = velocityBuffer[1];
        } else if (buttonLong) {
            targetVelocity = velocityBuffer[2];
        }

        if (allOff || ballsLaunched == 3 && !empty) {
            if (allOff) {
                ballsLaunched = 0;
                // eventually we will add logic to detect if there are balls in the shooter
                empty = false;
            } else {
                empty = true;

            }

            stopShooterMotor();
            stopShoot();
            intake.stopIntake();
            ShooterState = ShooterState.IDLE;

        } else if (anyOn) {
            double currentVel = shooterMotor.getVelocity();
            double currentVelDeg = shooterMotor.getVelocity(AngleUnit.DEGREES);
            log.d("Shooter", "-----------------------------------");
            log.d("Shooter", "Velocity (rag): " + currentVel);
            log.d("Shooter", "Velocity (deg): " + currentVelDeg);
            log.d("Shooter", "Target Velocity: " + targetVelocity);
            log.d("Shooter", "State: " + ShooterState.toString());
            log.d("Shooter", "Balls Launched: " + ballsLaunched);
            if(ShooterState == ShooterState.IDLE) {
                startShooterMotor(targetVelocity);
                ShooterState = ShooterState.SPINNING_UP;
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) > allowedError) {
                if (lastShooterState == ShooterState.SHOOTING) {
                    ballsLaunched++;
                    lastShooterState = ShooterState.SPINNING_UP;
                }

                startShooterMotor(targetVelocity);
               //
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) < allowedError) {
                ShooterState = ShooterState.SHOOTING;
                startShoot();
                intake.startIntake(1.0);
            } else if (ShooterState == ShooterState.SHOOTING && (targetVelocity - currentVel) > allowedError) {
                startShooterMotor(targetVelocity);
                stopShoot();
                lastShooterState = ShooterState.SHOOTING;
                ShooterState = ShooterState.SPINNING_UP;



            }
        }
    }

    public void startIntake() {
        if(ShooterState == ShooterState.IDLE)
            intake.startIntake(1.0);
    }

    public void stopIntake() {
        if(ShooterState == ShooterState.IDLE)
            intake.stopIntake();
    }

    public void reverseIntake() {
        if(ShooterState == ShooterState.IDLE)
            intake.startIntake(-1.0);
    }
    public int getBallsLaunched() {
        return ballsLaunched;
    }
}

