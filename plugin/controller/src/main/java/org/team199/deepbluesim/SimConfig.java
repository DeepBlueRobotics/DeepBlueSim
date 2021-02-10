package org.team199.deepbluesim;

import org.team199.deepbluesim.Constants.Preset;

public class SimConfig extends BaseSimConfig {

    public static void initConfig() {
        if (Constants.preset == Preset.SWERVE) {
            for (int i = 0; i < Constants.turnPorts.length; i++) {
                setMotorEPR("Talon SRX[" + Constants.turnPorts[i] + "]", Constants.pg71EPR);
                setMotorMaxEncVoltage("Talon SRX[" + Constants.turnPorts[i] + "]", Constants.maxEncVoltages[i]);
            }
        }
    }

}