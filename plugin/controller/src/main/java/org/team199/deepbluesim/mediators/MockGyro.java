package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.Gyro;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;

/**
 * Handles the linking of the simulated AHRS gyro to Webots
 * @see com.kauailabs.navx.frc.AHRS
 */
public final class MockGyro implements Runnable {

    private static boolean gyroCreated = false;
    private static SimDeviceSim gyroSim;
    private static Gyro webotsGyro;
    private static double angle = 0;

    /**
     * Links the simulated AHRS gyro to Webots if it has not been already
     */
    public static void linkGyro() {
        if(gyroCreated) {
            return;
        }
        gyroCreated = true;
        // Create Sims
        gyroSim = new SimDeviceSim("navX-Sensor[0]");
        webotsGyro = Simulation.getRobot().getGyro("gyro");
        if(webotsGyro != null) {
            webotsGyro.enable(Constants.sensorTimestep);
            Simulation.registerPeriodicMethod(new MockGyro());
        }
    }

    @Override
    public void run() {
        /* getValues() returns angular speeds about each axis (x, y, z).
           reading represents the change in angular position about the z axis.
           getValues()[2] is negated to convert from Webots's coordinate system (counter-clockwise = positive) to WPILib's coordinate system (counter-clockwise = negative).
        */
        double reading = -webotsGyro.getValues()[2] * Simulation.getBasicTimeStep();
        // In testing, reading was sometimes NAN in the first second of the simulation.
        // Also convert from radians to degrees
        angle += Double.isNaN(reading) ? 0 : (180 * reading / Math.PI);
        // Make sure angle is between 0 and 359 inclusive
        // angle = Math.copySign(Math.abs(angle) % 360, angle);
        // Update the WPIlib gyro
        gyroSim.set("Yaw", angle + "");
    }

    private MockGyro() {}

}