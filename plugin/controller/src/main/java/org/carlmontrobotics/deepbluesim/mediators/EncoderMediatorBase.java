package org.carlmontrobotics.deepbluesim.mediators;

import org.carlmontrobotics.deepbluesim.Constants;
import org.carlmontrobotics.deepbluesim.Simulation;

import com.cyberbotics.webots.controller.PositionSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public abstract class EncoderMediatorBase implements Runnable {

    public final PositionSensor encoder;
    public final boolean isOnMotorShaft;
    public final boolean isAbsolute;
    public final double absoluteOffsetDeg;
    public final boolean isInverted;
    public final int countsPerRevolution;
    public final double gearing;

    private double lastPositionRad = 0;

    public EncoderMediatorBase(PositionSensor encoder) {
        this(encoder, false, false, 0, false, 0, 1); // TODO: get these values from the robot
    }

    public EncoderMediatorBase(PositionSensor encoder, boolean isOnMotorShaft, boolean isAbsolute, double absoluteOffsetDeg, boolean isInverted, int countsPerRevolution, double gearing) {
        this.encoder = encoder;
        this.isOnMotorShaft = isOnMotorShaft;
        this.isAbsolute = isAbsolute;
        this.absoluteOffsetDeg = absoluteOffsetDeg;
        this.isInverted = isInverted;
        this.countsPerRevolution = countsPerRevolution;
        this.gearing = gearing;

        encoder.enable(Constants.sensorTimestep);
        Simulation.registerPeriodicMethod(this);
    }

    @Override
    public void run() {
        double positionRad = encoder.getValue();

        if(isOnMotorShaft) {
            positionRad *= gearing;
        }

        if(isAbsolute) {
            positionRad += Units.degreesToRadians(absoluteOffsetDeg);
            MathUtil.inputModulus(positionRad, 0, Math.PI * 2);
        }

        if(isInverted) {
            positionRad *= -1;
        }

        double velocityRadPerSec = (positionRad - lastPositionRad) / Simulation.getBasicTimeStep();
        lastPositionRad = positionRad;

        setPosition((int) Math.round(Units.radiansToRotations(positionRad) * countsPerRevolution));
        setVelocity((int) Math.round(Units.radiansToRotations(velocityRadPerSec) * countsPerRevolution));
    }

    public abstract void setPosition(int positionCounts);

    public abstract void setVelocity(int velocityCountsPerSecond);

}
