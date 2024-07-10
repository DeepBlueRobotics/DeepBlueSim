package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.wpiws.devices.PWMSim;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Node;

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

        if (motor.getBrake() != null) {
            // gearing^2*k_T/(R*k_v)
            double dampingConstant = gearing * gearing
                    * motorConstants.KtNMPerAmp / (motorConstants.rOhms
                            * motorConstants.KvRadPerSecPerVolt);
            motor.getBrake().setDampingConstant(dampingConstant);
        }

        motorDevice.registerSpeedCallback((deviceName, currentOutput) -> {
            double velocity = currentOutput * motorConstants.freeSpeedRadPerSec;
            double maxTorque = gearing * motorConstants.KtNMPerAmp
                    / (motorConstants.rOhms * motorConstants.KvRadPerSecPerVolt)
                    * motorConstants.freeSpeedRadPerSec;
            switch (motor.getNodeType()) {
                case Node.ROTATIONAL_MOTOR:
                    motor.setAvailableTorque(
                            Math.abs(currentOutput) * maxTorque);
                    break;
                case Node.LINEAR_MOTOR:
                    motor.setAvailableForce(
                            Math.abs(currentOutput) * maxTorque);
                    break;
                default:
                    throw new UnsupportedOperationException(
                            "Unsupported motor node type %d. Must be either a RotationalMotor or a LinearMotor: "
                                    .formatted(motor.getNodeType()));
            }
            motor.setVelocity((inverted ? -1 : 1) * velocity / gearing);
        }, true);
    }

}
