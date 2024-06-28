package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.deepbluesim.Simulation;
import org.carlmontrobotics.wpiws.devices.CANAnalogInputSim;

import com.cyberbotics.webots.controller.DistanceSensor;

public class PlayingWithFusionTimeOfFlightMediator implements Runnable {

    // We can't set this via the SimDevice
    // See comment in run()
    public static final int SAMPLING_PERIOD_MS = 100;

    public final DistanceSensor sensor;
    public final CANAnalogInputSim rangeDevice, ambientLightLevelDevice;

    public PlayingWithFusionTimeOfFlightMediator(DistanceSensor sensor,
            CANAnalogInputSim rangeDevice,
            CANAnalogInputSim ambientLightLevelDevice) {
        this.sensor = sensor;
        this.rangeDevice = rangeDevice;
        this.ambientLightLevelDevice = ambientLightLevelDevice;

        sensor.enable(100);

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
