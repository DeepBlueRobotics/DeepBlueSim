package org.team199.deepbluesim.mediators;

import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.deepbluesim.BaseSimConfig;
import org.team199.deepbluesim.Simulation;

public class MockedPhoenixSensor implements Runnable {
    private String name;
    private SimDeviceSim sim;
    private PositionSensor webotsEncoder;
    private int edgesPerRevolution;

    public MockedPhoenixSensor(SimDeviceSim sim, String motorID) {
        this.name = "PWM" + motorID;
        this.sim = sim;
        webotsEncoder = Simulation.getRobot().getMotor(name).getPositionSensor();
        edgesPerRevolution = 80;

        if(webotsEncoder != null) {
            webotsEncoder.enable(BaseSimConfig.getSensorTimestep());
            Simulation.registerPeriodicMethod(this);
        }
    }

    public void setEdgesPerRevolution(int edgesPerRevolution) {
        this.edgesPerRevolution = edgesPerRevolution;
    }

    @Override
    public void run() {
        // Get the position of the Webots encoders and set the position of the WPIlib encoders 
        // getValue() returns radians
        // revoultions = radians * gearing / pi
        double revolutions = (webotsEncoder.getValue() * BaseSimConfig.getMotorGearing(name)) / Math.PI;
        int edges = (int) Math.floor(revolutions * edgesPerRevolution);
        sim.set("edges", "" + edges);
    }
}