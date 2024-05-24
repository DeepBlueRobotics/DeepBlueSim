package org.team199.deepbluesim.mediators;

import org.team199.wpiws.devices.AnalogInputSim;

import com.cyberbotics.webots.controller.PositionSensor;

public class AnalogInputEncoderMediator extends EncoderMediatorBase {

    public final AnalogInputSim device;

    public AnalogInputEncoderMediator(PositionSensor encoder, AnalogInputSim device, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        super(encoder, isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        this.device = device;
    }

    @Override
    public void setPosition(int positionCounts) {
        // 1 Volt = 1 Rotation. The robot code can use
        // SparkAnalogSensor.setPositionConversionFactor() to change what
        // SparkAnalogSensor.getPosition() returns.
        device.setVoltage(1.0 * positionCounts / countsPerRevolution);
    }

    @Override
    public void setVelocity(int velocityCountsPerSecond) {
        // AnalogInput data doesn't include velocity so we do nothing.
    }

}
