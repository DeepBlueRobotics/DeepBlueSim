package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.deepbluesim.BaseSimConfig;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;

/**
 * Forwards motor calls from WPILib motor controllers to Webots
 */
public class MockSparkMotor implements Runnable {

    private Motor motor;
    private PositionSensor webotsEncoder;
    private SimDeviceSim simMotor;
    private String motorName;
    private double currentOutput;
    private boolean hasEncoder;

    public static void linkMotor(String motorName) {
        Simulation.registerPeriodicMethod(new MockSparkMotor(motorName));
    }

    /**
     * Creates a new MockCANMotor 
     * @param robot the Webots robot
     * @param motorName the name of the Webots motor to which to connect
     */
    public MockSparkMotor(String motorName) {
        this.motorName = motorName;

        motor = Simulation.getRobot().getMotor(motorName);
        simMotor = new SimDeviceSim(motorName);
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);
        currentOutput = 0;

        webotsEncoder = motor.getPositionSensor();
        if(webotsEncoder != null) {
            webotsEncoder.enable(BaseSimConfig.getSensorTimestep());
            hasEncoder = true;
        } else hasEncoder = false;
    }

    @Override
    public void run() {
        currentOutput = Double.parseDouble(simMotor.get("Applied Output"));
        motor.setVelocity(motor.getMaxVelocity() * currentOutput);
        
        if (hasEncoder) {
            // Get the position of the Webots encoders and set the position of the WPIlib encoders 
            // getValue() returns radians
            // revoultions = radians * gearing / (2 * pi)
            double revolutions = (webotsEncoder.getValue() / BaseSimConfig.getMotorGearing(motorName)) / (2 * Math.PI);
            // Sensor resolution for a SPARK MAX quadrature encoder is 4096 sensor units per revolution
            simMotor.set("Position", revolutions * 4096);
        }
    }

}