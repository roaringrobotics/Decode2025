package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.TelemetryI;

public class TelemetryHelper {

    public static void UpdateTelemetry(TelemetryI telemetry, Object... args)
    {
        if(args.length % 2 != 0) {
            // Args must be in pairs.
            throw new RuntimeException("TelemetryHelper invalid args");
        }
        for(int i = 0; i < args.length; i+=2) {
            telemetry.addData((String) args[i], args[i + 1]);
        }
        telemetry.update();
    }
}
