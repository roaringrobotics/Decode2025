package org.firstinspires.ftc.teamcode.Interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface TelemetryI {
    Telemetry.Item addData(String caption, Object value);

    void update();
}
