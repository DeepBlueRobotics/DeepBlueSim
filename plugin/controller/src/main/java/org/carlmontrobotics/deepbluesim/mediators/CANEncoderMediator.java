package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.wpiws.devices.CANEncoderSim;

import com.cyberbotics.webots.controller.PositionSensor;

public class CANEncoderMediator extends EncoderMediatorBase {

    public final CANEncoderSim device;

    public CANEncoderMediator(PositionSensor encoder, CANEncoderSim device, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        super(encoder, isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        this.device = device;
    }

    @Override
    public void setPosition(int positionCounts) {
        device.setPosition(1.0 * positionCounts / countsPerRevolution);
    }

    @Override
    public void setVelocity(int velocityCountsPerSecond) {
        device.setVelocity(1.0 * velocityCountsPerSecond / countsPerRevolution);
    }

}
