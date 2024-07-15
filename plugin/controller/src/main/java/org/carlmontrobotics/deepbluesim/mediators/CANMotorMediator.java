package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.deepbluesim.Simulation;
import org.carlmontrobotics.wpiws.devices.CANEncoderSim;
import org.carlmontrobotics.wpiws.devices.CANMotorSim;

import com.cyberbotics.webots.controller.Brake;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.PositionSensor;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Links WPILib motor controllers to Webots
 */
public class CANMotorMediator implements Runnable {

    public static final int NEO_BUILTIN_ENCODER_CPR = 42;

    public final Motor motor;
    public final double gearing;
    public final boolean inverted;
    public final DCMotor motorConstants;
    public final CANMotorSim motorDevice;
    public final Brake brake;
    public final PositionSensor positionSensor;

    private double requestedOutput = 0;
    private double actualOutput = 0;
    private boolean brakeMode = true;
    private double neutralDeadband = 0.04;
    private double dampingConstant = 0.0;
    private double lastPosRads = Double.NaN;

    /**
     * Creates a new CANMotorMediator
     * 
     * @param motor the Webots motor to link to
     * @param simDevice the SimDeviceSim to use
     * @param motorConstants the motor constants to use
     * @param gearing the gear reduction ratio to use. When used with a LinearMotor this includes
     *        the conversion between meters and radians.
     * @param inverted whether positive voltage should result in CW rotation (true) or CCW rotation
     *        (false).
     */
    public CANMotorMediator(Motor motor, CANMotorSim simDevice, DCMotor motorConstants, double gearing, boolean inverted) {
        this.motor = motor;
        motorDevice = simDevice;
        this.motorConstants = motorConstants;
        this.gearing = gearing;
        this.inverted = inverted;

        if(motor.getName().startsWith("DBSim_Motor_Spark")) {
            PositionSensor encoder = motor.getPositionSensor();
            if(encoder == null) {
                System.err.println(String.format("WARNING: Spark encoder not found for motor: \"%s\", no position data will be reported!", motor.getName()));
            } else {
                new CANEncoderMediator(encoder, new CANEncoderSim(motorDevice.id.replace("CANMotor:", "CANEncoder:"), "SimDevice"), true, false, 0, inverted, NEO_BUILTIN_ENCODER_CPR, gearing);
            }
        }

        // Bus voltage is not motor specific. It would need to be simulated by a simulated PDP/PDH.
        // Since we don't simulate it, we set it to 0.0 to be compliant with the spec.
        motorDevice.setBusvoltage(0.0);

        // We simulate motor current (see the run() method below), not supply current, so we set
        // supply current to 0.0 to be compliant with the spec.
        motorDevice.setSupplycurrent(0.0);

        this.brake = motor.getBrake();
        if(brake == null) {
            System.err.println(String.format("WARNING: Brake not found for motor: \"%s\", braking will be disabled!", motor.getName()));
        }

        this.positionSensor = motor.getPositionSensor();
        if (positionSensor == null) {
            System.err.println(String.format(
                    "WARNING: Encoder not found for motor: \"%s\", current will not be reported!",
                    motor.getName()));
        }

        // Use velocity control
        motor.setPosition(Double.POSITIVE_INFINITY);
        if (brake != null) {
            // gearing^2*k_T/(R*k_v)
            dampingConstant = gearing * gearing * motorConstants.KtNMPerAmp
                    / (motorConstants.rOhms
                            * motorConstants.KvRadPerSecPerVolt);
            brake.setDampingConstant(dampingConstant);

        }
        motorDevice.registerBrakemodeCallback((name, enabled) -> {
            brakeMode = enabled;
            updateBrakeDampingConstant();
        }, true);
        motorDevice.registerNeutraldeadbandCallback((name, deadband) -> {
            neutralDeadband = deadband;
            updateBrakeDampingConstant();
        }, true);
        motorDevice.registerPercentoutputCallback((name, percentOutput) -> {
            requestedOutput = percentOutput;
            updateBrakeDampingConstant();
        }, true);

        Simulation.registerPeriodicMethod(this);
    }

    private void updateBrakeDampingConstant() {
        actualOutput = requestedOutput;
        if (Math.abs(actualOutput) < neutralDeadband) {
            actualOutput = 0.0;
        }
        if (brake != null) {
            if (!brakeMode && actualOutput == 0.0) {
                brake.setDampingConstant(0);
            } else {
                brake.setDampingConstant(dampingConstant);
            }
        }
        double velocity = actualOutput * motorConstants.freeSpeedRadPerSec;
        double maxTorque = gearing * motorConstants.stallTorqueNewtonMeters;
        switch (motor.getNodeType()) {
            case Node.ROTATIONAL_MOTOR:
                motor.setAvailableTorque(
                        Math.abs(actualOutput) * maxTorque);
                break;
            case Node.LINEAR_MOTOR:
                motor.setAvailableForce(
                        Math.abs(actualOutput) * maxTorque);
                break;
            default:
                throw new UnsupportedOperationException(
                        "Unsupported motor node type %d. Must be either a RotationalMotor or a LinearMotor: "
                                .formatted(motor.getNodeType()));
        }
        motor.setVelocity((inverted ? -1 : 1) * velocity / gearing);
    }


    @Override
    public void run() {
        if (positionSensor == null) {
            return;
        }
        double samplingPeriodMs = positionSensor.getSamplingPeriod();
        if (samplingPeriodMs == 0) {
            // Sensor was not enabled.
            return;
        }
        double posRads = positionSensor.getValue();
        if (!Double.isNaN(lastPosRads)) {
            double velRadPerSec = (posRads - lastPosRads)
                    / (samplingPeriodMs / 1000.0) * gearing;
            // i = (V-w/k_v)/R
            double currentDraw =
                    (actualOutput * motorConstants.nominalVoltageVolts
                            - velRadPerSec / motorConstants.KvRadPerSecPerVolt)
                            / motorConstants.rOhms;
            // Arguably, we should be able to set a negative motor current, but the current spec
            // says it needs to be >= 0.0
            // TODO: Create a wpilib issue to address this.
            currentDraw = Math.abs(currentDraw);
            motorDevice.setMotorcurrent(currentDraw);
        }
        lastPosRads = posRads;
    }

}