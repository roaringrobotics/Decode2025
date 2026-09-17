package org.firstinspires.ftc.teamcode.Pathing;

public class PidParamSet
{
    PidParamSet(PidParams l, PidParams h)
    {
        location = l;
        heading = h;
    }
    public PidParams location;
    public PidParams heading;
}
