package Fakes;

import org.firstinspires.ftc.teamcode.Interfaces.LogI;

public class FakeLog implements LogI {
    @Override
    public void d(String tag, String message) {
        System.out.println(tag + ": " + message);
    }
}