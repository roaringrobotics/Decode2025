package org.firstinspires.ftc.teamcode.Pathing;

import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

// Used to limit rate of change in power to motors.
public class PowerRampController {
    public double maxChange;
    public double lastValue;
    public double lastTimeSetMs;
    public double minTimeMs = 10;
    TimeSourceI timeSource;

    public PowerRampController(double maxChange, TimeSourceI ts)
    {
        this.maxChange = maxChange;
        timeSource = ts;
        lastTimeSetMs = Long.MIN_VALUE;
    }

    public void Reset()
    {
        lastTimeSetMs = Long.MIN_VALUE;
    }

    public double getValue(double newValue)
    {
        timeSource.update();
        double currentTime = timeSource.currentTimeMillis();
        double deltaTime = Math.abs(currentTime - lastTimeSetMs);

        if(deltaTime < minTimeMs) {
            // Wait at least minTimeSec seconds before changing again.
            // Increasing from 0 to 1 power will take 200 milisecs (0.2 seconds)
            // if minTimeMs = 20 and maxChange = 0.1
            return lastValue;
        }
        double delta = newValue - lastValue;
        double sign = Math.signum(delta);

        if(Math.abs(delta) > maxChange)
        {
            delta = maxChange*sign;
        }

        lastValue = lastValue + delta;
        lastTimeSetMs = currentTime;
        return lastValue;
    }
}
