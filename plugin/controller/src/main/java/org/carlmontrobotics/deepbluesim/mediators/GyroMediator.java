package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.deepbluesim.Constants;
import org.carlmontrobotics.deepbluesim.Simulation;
import org.carlmontrobotics.wpiws.devices.SimDeviceSim;

import com.cyberbotics.webots.controller.Gyro;

/**
 * Handles the linking of the simulated AHRS gyro to Webots
 * @see com.kauailabs.navx.frc.AHRS
 */
public class GyroMediator implements Runnable {

    public final Gyro gyro;
    public final SimDeviceSim device;
    private double angle = 0;

    /**
     * Links the simulated AHRS gyro to Webots
     * @param gyro the Webots gyro to link to
     */
    public GyroMediator(Gyro gyro) {
        this.gyro = gyro;
        gyro.enable(Constants.sensorTimestep);
        device = new SimDeviceSim("navX-Sensor[0]");
        Simulation.registerPeriodicMethod(this);
    }

    @Override
    public void run() {
        /* getValues() returns angular speeds about each axis (x, y, z).
           reading represents the change in angular position about the z axis.
           getValues()[2] is negated to convert from Webots's coordinate system (counter-clockwise = positive) to WPILib's coordinate system (counter-clockwise = negative).
        */
        double reading = -gyro.getValues()[2] * Simulation.getBasicTimeStep();
        // In testing, reading was sometimes NAN in the first second of the simulation.
        // Also convert from radians to degrees
        angle += Double.isNaN(reading) ? 0 : (180 * reading / Math.PI);
        // Make sure angle is between 0 and 359 inclusive
        // angle = Math.copySign(Math.abs(angle) % 360, angle);
        // Update the WPIlib gyro
        device.set("Yaw", angle + "");
    }

}
