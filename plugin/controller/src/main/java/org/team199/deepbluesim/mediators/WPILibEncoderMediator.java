package org.team199.deepbluesim.mediators;

import org.team199.wpiws.devices.EncoderSim;

import com.cyberbotics.webots.controller.PositionSensor;

public class WPILibEncoderMediator extends EncoderMediatorBase {

    public final EncoderSim device;

    public WPILibEncoderMediator(PositionSensor encoder, EncoderSim device) {
        super(encoder);
        this.device = device;
    }

    public WPILibEncoderMediator(PositionSensor encoder, EncoderSim device, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        super(encoder, isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        this.device = device;
    }

    @Override
    public void setPosition(int positionCounts) {
        device.setCount(positionCounts);
    }

    @Override
    public void setVelocity(int velocityCountsPerSecond) {
        device.setPeriod(1.0D / velocityCountsPerSecond);
    }

}
