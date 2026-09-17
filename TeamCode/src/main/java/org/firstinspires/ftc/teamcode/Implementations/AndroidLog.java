package org.firstinspires.ftc.teamcode.Implementations;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Interfaces.LogI;

public class AndroidLog implements LogI {
    @Override
    public void d(String tag, String message) {
        Log.d(tag, message);
    }
}
