package org.team199.deepbluesim;

import java.util.concurrent.CopyOnWriteArrayList;

import com.cyberbotics.webots.controller.Robot;

/**
 * Manages control over the robot simulation and Webots connection
 */
public final class Simulation {

    private static boolean init = false;
    /**
     * An object representing the Webots robot
     */
    private static Robot robot;
    /**
     * The value of the basicTimeStep field of the WorldInfo node of the Webots robot
     * @see Robot#getBasicTimeStep()
     */
    private static double timeStep;
    /**
     * {@link #timeStep} converted into milliseconds. This is equivalent to <code>timeStep * 1000</code>
     */
    private static double timeStepMillis;
    // Use a CopyOnWriteArrayList to prevent syncronization errors
    private static final CopyOnWriteArrayList<Runnable> periodicMethods;

    static {
        periodicMethods = new CopyOnWriteArrayList<>();
        // Register callbacks
        SimRegisterer.init();
    }

    public static synchronized void init(Robot robot, double basicTimeStepMillis) {
        if(init) {
            return;
        }
        Simulation.robot = robot;
        timeStepMillis = basicTimeStepMillis;
        timeStep = (double)basicTimeStepMillis / 1000;
        init = true;
    }

    /**
     * Registers a method to be run as part of the robot's periodic loop
     * @param method
     */
    public static void registerPeriodicMethod(Runnable method) {
        periodicMethods.add(method);
    }

    public static Robot getRobot() {
        return robot;
    }

    public static int getBasicTimeStep() {
        return timeStep;
    }

    public static double getTimeStepMillis() {
        return timeStepMillis;
    }

    public static void runPeriodicMethods() {
        periodicMethods.forEach(Runnable::run);
    }

    private Simulation() {}

}