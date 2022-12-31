package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.StringCallback;

public class MockedCANCoder implements Runnable, StringCallback {

    private SimDeviceSim device;
    private PositionSensor webotsEncoder;
    private double gearing;

    public MockedCANCoder(SimDeviceSim device, String name) {
        this.device = device;
        gearing = 1;
        device.registerValueChangedCallback("gearing", this, true);
        webotsEncoder = Simulation.getRobot().getPositionSensor(name);
        if(webotsEncoder != null) {
            webotsEncoder.enable(Constants.sensorTimestep);
            Simulation.registerPeriodicMethod(this);
        }
    }

    @Override
    public void callback(String name, String value) {
        if(value == null) return; // Value has not yet been set
        try {
            gearing = Double.parseDouble(value);
        } catch(NumberFormatException e) {}
    }

    @Override
    public void run() {
        if(webotsEncoder == null) return;
        // Get the position of the Webots encoders and set the position of the WPIlib encoders 
        // getValue() returns radians
        // revoultions = radians * gearing / 2pi
        double revolutions = (webotsEncoder.getValue() * gearing) / (2*Math.PI);
        device.set("count", revolutions);
    }

}
