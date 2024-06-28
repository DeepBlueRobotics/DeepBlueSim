package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.wpiws.devices.PWMSim;

import com.cyberbotics.webots.controller.Motor;

import edu.wpi.first.math.system.plant.DCMotor;

public class PWMMotorMediator {

    public final Motor motor;
    public final double gearing;
    public final boolean inverted;
    public final DCMotor motorConstants;
    public final PWMSim motorDevice;

    public PWMMotorMediator(Motor motor, PWMSim simDevice, DCMotor motorConstants, double gearing, boolean inverted) {
        this.motor = motor;
        this.motorDevice = simDevice;
        this.motorConstants = motorConstants;
        this.gearing = gearing;
        this.inverted = inverted;

        // Use velocity control
        motor.setPosition(Double.POSITIVE_INFINITY);

        // Disable braking
        if(motor.getBrake() != null) motor.getBrake().setDampingConstant(0);

        motorDevice.registerSpeedCallback((deviceName, speed) -> {
            double velocity = speed * motorConstants.freeSpeedRadPerSec;
            motor.setVelocity((inverted ? -1 : 1) * velocity / gearing);
        }, true);
    }

}
