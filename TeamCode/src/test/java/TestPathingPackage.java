import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.FakeOpMode;
import org.firstinspires.ftc.teamcode.Interfaces.OpModeI;
import org.firstinspires.ftc.teamcode.Pathing.PathController;
import org.firstinspires.ftc.teamcode.Pathing.PidParamCollection;
import org.firstinspires.ftc.teamcode.Pathing.PidParams;
import org.junit.Test;

import Fakes.FakeHardware;
import Fakes.FakeLog;
import Fakes.FakeTimeSource;

public class TestPathingPackage {

    @Test
    public void TestDriveForward()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new FakeTimeSource(), new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(0);
        pathing.setTargetLocation(0,24);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);
    }

    @Test
    public void TestDriveLeft()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new FakeTimeSource(), new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(0);
        pathing.setTargetLocation(24,0);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);
    }

    @Test
    public void TestDriveDiagonal()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new FakeTimeSource(), new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(0);
        pathing.setTargetLocation(24,24);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);
    }

    @Test
    public void TestDriveRotation()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new FakeTimeSource(), new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(90);
        pathing.setTargetLocation(0,0);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);
    }

    @Test
    public void TestDriveRotationAndLocationSequential()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        FakeTimeSource timeSource = new FakeTimeSource();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                timeSource, new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(90);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);

        pathing.setTargetLocation(0,10);
        result = pathing.run(20);
        timeSource.resetTotal();
        assertTrue("pathing.run", result);

    }

    @Test
    public void TestDriveLocationAndRotation()
    {
        FakeHardware hw = new FakeHardware();
        hw.resetImu();
        OpModeI opMode = new FakeOpMode();
        PathController pathing = new PathController(
                hw, opMode, 0.9,
                PidParamCollection.ParamSetName.UNIT_TEST_SIM,
                new FakeTimeSource(), new FakeLog());
        // Move forward 24 inches.
        pathing.setTargetHeadingDeg(-90);
        pathing.setTargetLocation(24,10);
        boolean result = pathing.run(10);
        assertTrue("pathing.run", result);
    }
}
