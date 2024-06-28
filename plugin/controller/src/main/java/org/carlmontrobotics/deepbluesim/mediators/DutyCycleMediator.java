package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.wpiws.devices.DutyCycleSim;

import com.cyberbotics.webots.controller.PositionSensor;

public class DutyCycleMediator extends EncoderMediatorBase {

    public final DutyCycleSim device;

    public DutyCycleMediator(PositionSensor encoder, DutyCycleSim device, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        super(encoder, isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        this.device = device;
        device.setConnected(true);
    }

    @Override
    public void setPosition(int positionCounts) {
        device.setPosition(1.0 * positionCounts / countsPerRevolution);
    }

    @Override
    public void setVelocity(int velocityCountsPerSecond) {
        // DutyCycle data doesn't include velocity so we do nothing.
    }

}
