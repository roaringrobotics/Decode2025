package Fakes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;

public class FakeTelemetry implements TelemetryI {

    @Override
    public Telemetry.Item addData(String caption, Object value) {
        return null;
    }

    @Override
    public void update() {

    }
}
