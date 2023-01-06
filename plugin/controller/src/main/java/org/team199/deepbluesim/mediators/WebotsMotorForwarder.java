package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;

import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.interfaces.DoubleCallback;
import org.team199.wpiws.interfaces.StringCallback;

/**
 * Forwards motor calls from WPILib motor controllers to Webots
 */
public class WebotsMotorForwarder implements DoubleCallback, Runnable, StringCallback {

    private double currentOutput, pos, timer;
    private Motor motor;
    private Node jointParameters;

    /**
     * Creates a new WebotsMotorForwarder
     * @param robot the Webots robot
     * @param motorName the name of the Webots motor to which to connect
     */
    public WebotsMotorForwarder(Robot robot, String motorName) {
        motor = robot.getMotor(motorName);
        currentOutput = 0;
        // Make sure that the motor can rotate any number of times
        if(motor != null) {
            motor.setPosition(Double.POSITIVE_INFINITY);
            motor.setVelocity(0);
            jointParameters = Supervisor.getSupervisorInstance().getFromDevice(motor).getParentNode();
            Simulation.registerPeriodicMethod(this);
        }
    }

    @Override
    public void callback(String name, String value) {
        if(value == null) return; // Value has not yet been set
        try {
            callback(name, Double.parseDouble(value));
        } catch(NumberFormatException e) {}
    }

    @Override
    public void callback(String name, double value) {
        currentOutput = value;
    }

    @Override
    public void run() {
        if(timer == 0) {
            timer = System.currentTimeMillis();
            return;
        }

        double velocity = motor.getMaxVelocity() * currentOutput;
        pos += velocity * (System.currentTimeMillis() - timer) / 1000;

        if(motor.getPositionSensor().getName().contains("CANCoder")) jointParameters.setJointPosition(pos, 1);
        else motor.setVelocity(velocity);

        timer = System.currentTimeMillis();
    }

}