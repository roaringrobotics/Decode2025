package org.firstinspires.ftc.teamcode.Pathing;

import java.util.Hashtable;

public class PidParamCollection {
    public enum ParamSetName {
        UNIT_TEST_SIM
    }

    public static Hashtable<ParamSetName, PidParamSet> paramsSets;

    static {
        paramsSets = new Hashtable<>();
        paramsSets.put(ParamSetName.UNIT_TEST_SIM,
                        new PidParamSet(
                            new PidParams(0.7, 0.0, 1.0),
                            new PidParams(0.01, 0.0, 0.0)));
    }
}

