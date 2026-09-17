package org.firstinspires.ftc.teamcode.Interfaces;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface HardwareI {
    void updateImuPos();
    void resetImu();

    void resetImuHeading();

    Pose2D getImuPose();
    double getImuHeading(AngleUnit unit);
    void setFrontLeftPower(double power);
    void setFrontRightPower(double power);
    void setBackLeftPower(double power);
    void setBackRightPower(double power);
    double getFrontLeftPower();
    double getFrontRightPower();
    double getBackLeftPower();
    double getBackRightPower();

    void updateState(TimeSourceI timeSource);

    void stopMotors();
}
