package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.deepbluesim.BaseSimConfig;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.devices.SimDeviceSim;

/**
 * Forwards motor calls from WPILib motor controllers to Webots
 */
public class MockCANMotor implements Runnable {

    private enum MotorType {
        SPARK, TALON, VICTOR;
    }

    private Motor motor;
    private PositionSensor webotsEncoder;
    private SimDeviceSim simMotor, encoder;
    private String motorName;
    private int unitsPerRevolution;
    private double currentOutput;
    private boolean hasEncoder;
    private MotorType type;

    public static void linkMotor(String motorName) {
        Simulation.registerPeriodicMethod(new MockCANMotor(motorName));
    }

    /**
     * Creates a new MockCANMotor 
     * @param robot the Webots robot
     * @param motorName the name of the Webots motor to which to connect
     */
    public MockCANMotor(String motorName) {
        this.motorName = motorName;

        motor = Simulation.getRobot().getMotor(motorName);
        simMotor = new SimDeviceSim(motorName);
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);

        String deviceName;
        if (motorName.startsWith("SPARK MAX")) {
            type = MotorType.SPARK;
            // Counts per revolution
            unitsPerRevolution = 4096;
            deviceName = motorName;
        }
        else if (motorName.startsWith("Talon SRX")) {
            type = MotorType.TALON;
            // Edges per revolution
            unitsPerRevolution = 80;
            deviceName = motorName + "/QuadEncoder";
        } else {
            // Assume the motor is a victor
            type = MotorType.VICTOR;
            // Edges per revolution
            unitsPerRevolution = 80;
            deviceName = motorName + "/QuadEncoder";
        }

        webotsEncoder = motor.getPositionSensor();
        if(webotsEncoder != null) {
            webotsEncoder.enable(BaseSimConfig.getSensorTimestep());
            encoder = new SimDeviceSim(deviceName);
            hasEncoder = true;
        } else hasEncoder = false;
    }

    @Override
    public void run() {
        try {
            if (type == MotorType.SPARK) currentOutput = Double.parseDouble(simMotor.get("Applied Output"));
            else currentOutput = Double.parseDouble(simMotor.get("percentOutput"));
        } catch (Exception e) { currentOutput = 0; }
        motor.setVelocity(motor.getMaxVelocity() * currentOutput);
        
        if (hasEncoder) {
            // Get the position of the Webots encoders and set the position of the WPIlib encoders 
            // getValue() returns radians
            // revoultions = radians * gearing / pi
            double revolutions = (webotsEncoder.getValue() * BaseSimConfig.getMotorGearing(motorName)) / Math.PI;
            int position = (int) Math.floor(revolutions * unitsPerRevolution);
            System.out.println(position);
            if (type == MotorType.SPARK) encoder.set("Position", position);
            else encoder.set("position", position);
        }
    }

}