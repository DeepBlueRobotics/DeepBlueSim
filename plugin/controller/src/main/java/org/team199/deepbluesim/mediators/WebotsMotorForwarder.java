package org.team199.deepbluesim.mediators;

import org.team199.wpiws.interfaces.DoubleCallback;
import org.team199.wpiws.interfaces.StringCallback;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

/**
 * Forwards motor calls from WPILib motor controllers to Webots
 */
public class WebotsMotorForwarder implements DoubleCallback, StringCallback {

    private Motor motor;

    /**
     * Creates a new WebotsMotorForwarder
     * @param robot the Webots robot
     * @param motorName the name of the Webots motor to which to connect
     */
    public WebotsMotorForwarder(Robot robot, String motorName) {
        motor = robot.getMotor(motorName);
        // Use velocity control
        motor.setPosition(Double.POSITIVE_INFINITY);
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
        motor.setVelocity(motor.getMaxVelocity() * value);
    }

}