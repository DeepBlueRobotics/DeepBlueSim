package org.team199.deepbluesim.mediators;

import java.util.Collection;

import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.devices.PWMSim;

import com.cyberbotics.webots.controller.Motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class PWMMotorMediator {

    public final Motor motor;
    public final double gearing;
    public final boolean inverted;
    public final DCMotor motorConstants;
    public final PWMSim motorDevice;

    public PWMMotorMediator(Motor motor, PWMSim simDevice, DCMotor motorConstants, double gearing, boolean inverted, Collection<ScopedObject<?>> callbackStore) {
        this.motor = motor;
        this.motorDevice = simDevice;
        this.motorConstants = motorConstants;
        this.gearing = gearing;
        this.inverted = inverted;

        // Use velocity control
        motor.setPosition(Double.POSITIVE_INFINITY);

        // Disable braking
        if(motor.getBrake() != null) motor.getBrake().setDampingConstant(0);

        callbackStore.add(motorDevice.registerSpeedCallback((deviceName, speed) -> {
            double velocity = speed * Units.radiansPerSecondToRotationsPerMinute(motorConstants.freeSpeedRadPerSec);
            motor.setVelocity((inverted ? -1 : 1) * velocity / gearing);
        }, true));
    }

}
