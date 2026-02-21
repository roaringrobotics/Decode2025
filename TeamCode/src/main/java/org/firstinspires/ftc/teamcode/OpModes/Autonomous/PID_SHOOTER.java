//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;
//import org.firstinspires.ftc.teamcode.RobotHardware.Intake;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.RobotHardware.Shooter;
////@Autonomous(name = "PID SHOOTER")
//
//public class PID_SHOOTER extends LinearOpMode {
//    double i = 0;
//    double p = 0.1;
//    double d = 0;
//    double f = 0;
//    double velocity = 1000;
//    private DcMotorEx shooterMotor;
//    private void Shooter(HardwareMap hardwareMap, Intake in, AndroidLog logIn) {
//        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooterMotor.setVelocityPIDFCoefficients(p, i, d, f);
//    }
//    public void runOpMode(){
//        shooterMotor.startShooterMotor(velocity);
//    }
//}
