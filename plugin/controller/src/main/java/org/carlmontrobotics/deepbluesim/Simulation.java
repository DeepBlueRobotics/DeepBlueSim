package org.carlmontrobotics.deepbluesim;

import java.util.concurrent.CopyOnWriteArrayList;

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;

/**
 * Manages control over the robot simulation and Webots connection
 */
public final class Simulation {

    private static boolean init = false;
    /**
     * An object representing the Webots robot
     */
    private static Supervisor robot;
    /**
     * The value of the basicTimeStep field of the WorldInfo node of the Webots robot
     * @see Robot#getBasicTimeStep()
     */
    private static double timeStep;
    /**
     * {@link #timeStep} converted into milliseconds. This is equivalent to <code>timeStep * 1000</code>
     */
    private static double timeStepMillis;
    // Use a CopyOnWriteArrayList to prevent synchronization errors
    private static final CopyOnWriteArrayList<Runnable> periodicMethods = new CopyOnWriteArrayList<>();

    public static synchronized void init(Supervisor robot, double basicTimeStepMillis) {
        if(init) {
            return;
        }
        Simulation.robot = robot;
        timeStepMillis = basicTimeStepMillis;
        timeStep = basicTimeStepMillis / 1000D;
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

    public static Supervisor getSupervisor() {
        return robot;
    }

    public static double getBasicTimeStep() {
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
