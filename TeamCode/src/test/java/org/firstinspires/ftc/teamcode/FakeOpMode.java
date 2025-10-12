package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.OpModeI;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

import Fakes.FakeTelemetry;

public class FakeOpMode implements OpModeI {
    @Override
    public boolean isStopRequested() {
        return false;
    }

    @Override
    public TelemetryI getTelemetry() {
        return new FakeTelemetry();
    }

    @Override
    public void updateState(TimeSourceI timeSource) {

    }
}
