package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;

public class MockedSparkEncoder implements Runnable {
    private String name;
    private SimDeviceSim encoder;
    private PositionSensor webotsEncoder;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;
    private double position;
    private double distancePerPulse;

    public MockedSparkEncoder(SimDeviceSim sim, String name) {
        this.name = name;
        encoder = sim;
        webotsEncoder = Simulation.getRobot().getPositionSensor(name);
        distancePerPulse = 1;
        sim.registerValueChangedCallback("distancePerPulse", (valueName, value) -> {
            if(value == null) return; // Value has not yet been set
            try {
                distancePerPulse = Double.parseDouble(value);
            } catch(NumberFormatException e) {}
        }, true);
        if(webotsEncoder != null) {
            webotsEncoder.enable(Constants.sensorTimestep);
            Simulation.registerPeriodicMethod(this);
        }
    }

    public double getPosition() {
        return position;
    }

    @Override
    public void run() {
        // Get the position of the Webots encoders and set the position of the WPIlib encoders 
        // getValue() returns radians
        // revoultions = radians * gearing / 2pi
        double revolutions = (webotsEncoder.getValue() * distancePerPulse) / (2*Math.PI);
        int count = (int) Math.floor(revolutions * countsPerRevolution);
        encoder.set("count", "" + count);
    }
}