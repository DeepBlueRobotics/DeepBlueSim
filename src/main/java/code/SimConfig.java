package code;

import code.lib.sim.BaseSimConfig;

public class SimConfig extends BaseSimConfig {

    public static void initConfig() {
        setDefaultMotorGearing(Constants.motorGearing);
    }

}