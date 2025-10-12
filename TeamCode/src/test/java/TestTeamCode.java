import static org.junit.Assert.assertEquals;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Pathing.PIDController;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.MockitoJUnitRunner;

import Fakes.FakeTimeSource;

@RunWith(MockitoJUnitRunner.class)
public class TestTeamCode {
    @Test
    public void testPid()
    {
        try (MockedStatic<Log> logMock = Mockito.mockStatic(Log.class)) {
            PIDController testPid = new PIDController(1, 0, 0);

            TimeSourceI timeSource = new FakeTimeSource();
            timeSource.update();
            double output;
            testPid.setTargetPoint(10);

            output = testPid.calculate(1, timeSource);
            timeSource.update();
            assertEquals(9.0, output, .01);
            output = testPid.calculate(15, timeSource);
            timeSource.update();
            assertEquals(-5.0, output, .01);
        }
    }

    @Test
    public void testVectorSubtractInPlace()
    {
        Pose2D lastPose = new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.DEGREES, 0);
        Vector2 target = new Vector2(24, 0);
        Vector2 result = new Vector2(0,0);
        result.subtractInPlace(target, lastPose);
        assertEquals(14, result.x, 0.01);
        assertEquals(-10, result.y, 0.01);
    }

    @Test
    public void testPowerRamp()
    {
        TimeSourceI timeSource = new FakeTimeSource();
        timeSource.update();
        PowerRampController controller = new PowerRampController(.1, timeSource);
        int count = 0;
        double value = 0.0;
        double targetValue = 1.0;

        while(Math.abs(value-targetValue) > 0.001)
        {
            timeSource.update();
            value = controller.getValue(targetValue);
            count++;
        }

        // expected is 19 here since the first getValue set the value and
        // doesn't need to wait the 20 ms
        assertEquals(19, count);
        assertEquals(1.0, controller.lastValue, .001);

        targetValue = 0;
        count = 0;
        while(Math.abs(value-targetValue) > 0.001)
        {
            timeSource.update();
            value = controller.getValue(targetValue);
            count++;
        }

        // expected is 20 here since the first decrease needs to wait
        // the 20 ms.
        assertEquals(20, count);
        assertEquals(0.0, controller.lastValue, .001);
    }
}
