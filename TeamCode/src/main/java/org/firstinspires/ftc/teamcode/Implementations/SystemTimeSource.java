package org.firstinspires.ftc.teamcode.Implementations;

import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

public class SystemTimeSource implements TimeSourceI {

    long time;
    long lastTime;
    long total;
    boolean initialized = false;

    @Override
    public void update() {
        lastTime = time;
        time = System.currentTimeMillis();

        if(!initialized)
        {
            lastTime = time;
            initialized = true;
        }

        total += deltaTimeMs();
    }

    @Override
    public long currentTimeMillis() {
        return time;
    }

    @Override
    public long totalRunningTimeMs() {
        return total;
    }

    @Override
    public long deltaTimeMs() {
        return time - lastTime;
    }

    @Override
    public void resetTotal() {
        total = 0;
    }

}
