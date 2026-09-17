package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transferball {

    private Servo servo;

    private static final double REST_POS = 0.0;
    private static final double FIRE_POS = 1.0;

    private long lastShotTime = 0;

    // Wiffle ball timing
    private static final long pushTime = 100;
    private static final long recoverTime = 120;

    public Transferball(HardwareMap hw) {
        servo = hw.get(Servo.class, "feeder");
        servo.setPosition(REST_POS);
    }

    public void fire() {
        long now = System.currentTimeMillis();

        // Push forward
        if (now - lastShotTime < pushTime) {
            servo.setPosition(FIRE_POS);
        }
        // Pull back
        else if (now - lastShotTime < pushTime + recoverTime) {
            servo.setPosition(REST_POS);
        }

        else {
            lastShotTime = now;
        }
    }

    public void reset() {
        servo.setPosition(REST_POS);
    }
}
