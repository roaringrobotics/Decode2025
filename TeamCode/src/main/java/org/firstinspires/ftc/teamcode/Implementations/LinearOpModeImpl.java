package org.firstinspires.ftc.teamcode.Implementations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.OpModeI;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;

public class LinearOpModeImpl implements OpModeI {
    LinearOpMode opMode;

    public LinearOpModeImpl(LinearOpMode opModeIn)
    {
        opMode = opModeIn;
    }
    @Override
    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }

    @Override
    public TelemetryI getTelemetry() {
        return new TelemetryWrapper(opMode.telemetry);
    }

    @Override
    public void updateState(TimeSourceI timeSource) {
    }
}
