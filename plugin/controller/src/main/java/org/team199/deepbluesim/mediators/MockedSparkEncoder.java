package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;

public class MockedSparkEncoder implements Runnable {
    private SimDeviceSim encoder;
    private PositionSensor webotsEncoder;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;
    private double gearing;

    public MockedSparkEncoder(SimDeviceSim sim, String name) {
        encoder = sim;
        webotsEncoder = Simulation.getRobot().getPositionSensor(name);
        gearing = 1;
        sim.registerValueChangedCallback("gearing", (valueName, value) -> {
            if(value == null) return; // Value has not yet been set
            try {
                gearing = Double.parseDouble(value);
            } catch(NumberFormatException e) {}
        }, true);
        if(webotsEncoder != null) {
            webotsEncoder.enable(Constants.sensorTimestep);
            Simulation.registerPeriodicMethod(this);
        }
    }

    @Override
    public void run() {
        // Get the position of the Webots encoders and set the position of the WPILib encoders
        // getValue() returns radians
        // revolutions = radians * gearing / 2pi
        double revolutions = (webotsEncoder.getValue() * gearing) / (2*Math.PI);
        int count = (int) Math.floor(revolutions * countsPerRevolution);
        encoder.set("count", "" + count);
    }
}