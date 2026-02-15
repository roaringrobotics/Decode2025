import static org.junit.Assert.assertEquals;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Interfaces.TimeSourceI;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Pathing.PIDController;
import org.firstinspires.ftc.teamcode.Pathing.PowerRampController;
import org.firstinspires.ftc.teamcode.RobotHardware.DriveTrain;
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

    @Test
    public void testFieldCentricPowerLevels() {
        try (MockedStatic<Log> logMock = Mockito.mockStatic(Log.class)) {
            // Case 1: forward (fieldX=1, fieldY=0, rotate=0, heading=0)
            DriveTrain.FieldCentricPowerLevels p = DriveTrain.computeFieldCentricPowerLevels(1.0, 0.0, 0.0, 0.0);
            assertEquals(1.0, p.frontLeftPower, 1e-6);
            assertEquals(1.0, p.backLeftPower, 1e-6);
            assertEquals(1.0, p.frontRightPower, 1e-6);
            assertEquals(1.0, p.backRightPower, 1e-6);

            // Case 2: strafe left (fieldX=0, fieldY=1, rotate=0, heading=0)
            p = DriveTrain.computeFieldCentricPowerLevels(0.0, 1.0, 0.0, 0.0);
            assertEquals(-1.0, p.frontLeftPower, 1e-6);
            assertEquals(1.0, p.backLeftPower, 1e-6);
            assertEquals(1.0, p.frontRightPower, 1e-6);
            assertEquals(-1.0, p.backRightPower, 1e-6);

            // Case 3: pure rotation (rotate=1)
            p = DriveTrain.computeFieldCentricPowerLevels(0.0, 0.0, 1.0, 0.0);
            assertEquals(1.0, p.frontLeftPower, 1e-6);
            assertEquals(1.0, p.backLeftPower, 1e-6);
            assertEquals(-1.0, p.frontRightPower, 1e-6);
            assertEquals(-1.0, p.backRightPower, 1e-6);
        }
    }

    @Test
    public void testVector2DistanceBetweenPoses() {
        Pose2D a = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D b = new Pose2D(DistanceUnit.INCH, 3, 4, AngleUnit.DEGREES, 0);
        double distance = Vector2.distanceBetweenPoses(a, b);
        assertEquals(5.0, distance, 1e-6);
    }

    @Test
    public void testVector2Normalize() {
        Vector2 v = new Vector2(3, 4);
        v.normalize();
        assertEquals(0.6, v.x, 1e-6);
        assertEquals(0.8, v.y, 1e-6);

        Vector2 v2 = new Vector2(0, 4);
        v2.normalize();
        assertEquals(0.0, v2.x, 1e-6);
        assertEquals(1.0, v2.y, 1e-6);

        Vector2 v3 = new Vector2(5, 0);
        v3.normalize();
        assertEquals(1.0, v3.x, 1e-6);
        assertEquals(0.0, v3.y, 1e-6);

    }
}
