package code.lib.sim;

import java.util.HashMap;

/**
 * Stores basic properties about how to configure the robot simulation
 */
public class BaseSimConfig {
    private static int sensorTimestep = 20;
    private static double defaultMotorGearing = 1;
    private static HashMap<String, Double> motorGearings = new HashMap<>();

    /**
     * Sets the sampling period to be used when enabling Webots sensors
     * @param timestep the sampling period
     * @see #getSensorTimestep()
     */
    protected static void setSensorTimestep(int timestep) {
        sensorTimestep = timestep;
    }

    /**
     * Retrieves the sampling period to be used when enabling Webots sensors
     * @see #setSensorTimestep(int)
     */
    protected static int getSensorTimestep() {
        return sensorTimestep;
    }

    /**
     * Sets the default motor gearing to be used when a specific one is not set
     * @param gearing the new default motor gearing
     * @see #setMotorGearing(String, double)
     * @see #getMotorGearing(String)
     */
    protected static void setDefaultMotorGearing(double gearing) {
        defaultMotorGearing = gearing;
    }

    /**
     * Sets the motor gearing for a specific motor
     * @param motor the name of the motor
     * @param diameter the gearing of the specified motor
     * @see #setDefaultMotorGearing(double)
     * @see #getMotorGearing(String)
     */
    protected static void setMotorGearing(String motor, double diameter) {
        motorGearings.put(motor, diameter);
    }

    /**
     * Retrieves the motor gearing for a specific motor
     * @param motor the name of the motor
     * @return the motor gearing for the specified motor
     * @see #setMotorGearing(String, double)
     * @see #setDefaultMotorGearing(double)
     */
    public static double getMotorGearing(String motor) {
        return motorGearings.containsKey(motor) ? motorGearings.get(motor) : defaultMotorGearing;
    }
}