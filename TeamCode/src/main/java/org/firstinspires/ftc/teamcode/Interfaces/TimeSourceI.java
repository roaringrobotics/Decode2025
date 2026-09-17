package org.firstinspires.ftc.teamcode.Interfaces;

public interface TimeSourceI {
    void update();
    long currentTimeMillis();
    long totalRunningTimeMs();
    long deltaTimeMs();

    void resetTotal();

    // Used for debugging.  Update to the next time step.
}
