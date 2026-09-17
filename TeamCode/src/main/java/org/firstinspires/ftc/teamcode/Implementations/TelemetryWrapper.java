package org.firstinspires.ftc.teamcode.Implementations;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;

public class TelemetryWrapper implements TelemetryI {
    Telemetry telem;
    public TelemetryWrapper(Telemetry telemetry) {
        telem = telemetry;
    }

    @Override
    public Telemetry.Item addData(String caption, Object value) {
        return telem.addData(caption, value);
    }

    @Override
    public void update() {
        telem.update();
    }
}
