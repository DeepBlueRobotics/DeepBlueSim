package org.team199.deepbluesim.mediators;

import org.team199.wpiws.devices.SimDeviceSim;

import com.cyberbotics.webots.controller.PositionSensor;

public class SimDeviceEncoderMediator extends EncoderMediatorBase {

    public final SimDeviceSim device;

    public SimDeviceEncoderMediator(PositionSensor encoder, SimDeviceSim device, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        super(encoder, isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        this.device = device;
    }

    @Override
    public void setPosition(int positionCounts) {
        device.set("Position", positionCounts);
    }

    @Override
    public void setVelocity(int velocityCountsPerSecond) {
        device.set("Velocity", velocityCountsPerSecond);
    }

}
