package org.team199.deepbluesim.mediators;

import java.util.Collection;

import org.team199.deepbluesim.ParseUtils;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.devices.SimDeviceSim;

import com.cyberbotics.webots.controller.Brake;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Links WPILib motor controllers to Webots
 */
public class SimDeviceMotorMediator implements Runnable {

    public static final int NEO_BUILTIN_ENCODER_CPR = 42;

    public final Motor motor;
    public final double gearing;
    public final boolean inverted;
    public final DCMotor motorConstants;
    public final SimDeviceSim motorDevice;
    public final Brake brake;

    private double requestedOutput = 0;
    private boolean brakeMode = true;
    private double neutralDeadband = 0.04;

    /**
     * Creates a new MotorMediator
     * @param motor the Webots motor to link to
     * @param simDevice the SimDeviceSim to use
     * @param motorConstants the motor constants to use
     * @param gearing the gear ratio to use
     * @param callbackStore a collection to store callbacks in
     */
    public SimDeviceMotorMediator(Motor motor, SimDeviceSim simDevice, DCMotor motorConstants, double gearing, boolean inverted, Collection<ScopedObject<?>> callbackStore) {
        this.motor = motor;
        motorDevice = simDevice;
        this.motorConstants = motorConstants;
        this.gearing = gearing;
        this.inverted = inverted;

        if(motor.getName().startsWith("DBSim_Motor_Spark Max")) {
            PositionSensor encoder = motor.getPositionSensor();
            if(encoder == null) {
                System.err.println(String.format("WARNING: Spark Max encoder not found for motor: \"%s\", no position data will be reported!", motor.getName()));
            } else {
                new SimDeviceEncoderMediator(encoder, new SimDeviceSim(motorDevice.id + "_RelativeEncoder"), true, false, 0, inverted, NEO_BUILTIN_ENCODER_CPR, gearing);
            }
        }

        this.brake = motor.getBrake();
        if(brake == null) {
            System.err.println(String.format("WARNING: Brake not found for motor: \"%s\", braking will be disabled!", motor.getName()));
        }

        // Use velocity control
        motor.setPosition(Double.POSITIVE_INFINITY);

        brake.setDampingConstant(motorConstants.stallTorqueNewtonMeters * gearing);

        callbackStore.add(motorDevice.registerValueChangedCallback("Brake Mode", (name, enabled) -> {
            brakeMode = Boolean.parseBoolean(enabled);
        }, true));
        callbackStore.add(motorDevice.registerValueChangedCallback("Neutral Deadband", (name, deadband) -> {
            neutralDeadband = Math.abs(ParseUtils.parseDoubleOrDefault(deadband, neutralDeadband));
        }, true));
        callbackStore.add(motorDevice.registerValueChangedCallback("Speed", (name, speed) -> {
            requestedOutput = ParseUtils.parseDoubleOrDefault(speed, requestedOutput);
        }, true));

        Simulation.registerPeriodicMethod(this);
    }

    @Override
    public void run() {
        // Apply the speed changes periodically so that changes to variables (ie brake mode) don't require a speed update to be applied
        // Copy requested output so that decreasing the neutral deadband can take effect without a speed update
        double currentOutput = requestedOutput;
        if(Math.abs(currentOutput) < neutralDeadband) {
            currentOutput = 0;
            brake.setDampingConstant(brakeMode ? motorConstants.stallTorqueNewtonMeters * gearing : 0);
        } else {
            brake.setDampingConstant(0);
        }

        double velocity = currentOutput * motorConstants.freeSpeedRadPerSec;
        motor.setVelocity((inverted ? -1 : 1) * velocity / gearing);

        double currentDraw = motorConstants.getCurrent(velocity, currentOutput * motorConstants.nominalVoltageVolts);
        motor.setAvailableTorque(motorConstants.getTorque(currentDraw) * gearing);

        motorDevice.set("Current Draw", currentDraw);
    }

}