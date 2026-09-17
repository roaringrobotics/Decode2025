package org.firstinspires.ftc.teamcode.Interfaces;


public interface OpModeI {
    boolean isStopRequested();
    TelemetryI getTelemetry();

    void updateState(TimeSourceI timeSource);
}
