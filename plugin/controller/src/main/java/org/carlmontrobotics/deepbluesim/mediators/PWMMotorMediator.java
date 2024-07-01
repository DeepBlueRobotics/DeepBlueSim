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

    /**
     * Creates a new PWMMotorMediator
     * 
     * @param motor the Webots motor to link to
     * @param simDevice the SimDeviceSim to use
     * @param motorConstants the motor constants to use
     * @param gearing the gear reduction ratio to use. When used with a LinearMotor this includes
     *        the conversion between meters and radians.
     * @param inverted whether positive voltage should result in CW rotation (true) or CCW rotation
     *        (false).
     */
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
