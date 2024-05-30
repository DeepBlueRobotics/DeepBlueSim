package org.team199.deepbluesim.mediators;

import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.AnalogInputSim;

import com.cyberbotics.webots.controller.DistanceSensor;

public class PlayingWithFusionTimeOfFlightMediator implements Runnable {

    public final DistanceSensor sensor;
    public final AnalogInputSim rangeDevice, ambientLightLevelDevice;

    public PlayingWithFusionTimeOfFlightMediator(DistanceSensor sensor,
            AnalogInputSim rangeDevice,
            AnalogInputSim ambientLightLevelDevice) {
        this.sensor = sensor;
        this.rangeDevice = rangeDevice;
        this.ambientLightLevelDevice = ambientLightLevelDevice;

        Simulation.registerPeriodicMethod(this);
    }

    @Override
    public void run() {
        rangeDevice.setVoltage(sensor.getValue()); // Millimeters

        // Other rangeDevice features are not yet exposed with AnalogInputSim API

        // Webots does not have a great way to get the ambient light level
        // We could use a camera and average all the pixels, but this seems like
        // overkill for a non-primary feature. For now, it is left unimplemented
    }

}
