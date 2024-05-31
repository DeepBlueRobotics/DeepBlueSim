package org.carlmontrobotics.libdeepbluesim;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

public class WebotsManager implements AutoCloseable {
    private final TimedRobot robot;
    private final Timer robotTime = new Timer();

    /**
     * Time value set by the simulator and the robot to indicate that the simulation should start.
     */
    private static final double START_SIMULATION = -2.0;

    @SuppressWarnings("resource")
    private final Notifier pauser = new Notifier(() -> {
        // This is replaced in waitForUserToStart()
    });

    public WebotsManager(TimedRobot robot) {
        this.robot = robot;
    }

    public double getTimeSecs() {
        return robotTime.get();
    }

    public Watcher addWatcher(String defPath) {
        return new Watcher(defPath);
    }

    public void waitForUserToStart(String worldFile) throws TimeoutException {
        // Start robot time right before the first periodic call is made so that we ignore
        // startup time.
        robotTime.stop();
        robotTime.reset();
        // Set the offset to -period to run before other WPILib periodic methods
        robot.addPeriodic(robotTime::start, robot.getPeriod(),
                -robot.getPeriod());
        SimDevice timeSynchronizer = SimDevice.create("TimeSynchronizer");
        SimDouble simTimeSecSim = timeSynchronizer.createDouble("simTimeSec",
                SimDevice.Direction.kInput, -1.0);
        final SimDouble robotTimeSecSim = timeSynchronizer.createDouble(
                "robotTimeSec", SimDevice.Direction.kOutput, -1.0);
        SimDeviceSim timeSynchronizerSim = new SimDeviceSim("TimeSynchronizer");

        pauser.setCallback(() -> {
            double simTimeSec = simTimeSecSim.get();
            double robotTimeSec = robotTime.get();
            double deltaSecs = simTimeSec - robotTimeSec;
            // If we still haven't caught up to the simulator, then wait longer.
            // This would typically happen when robot time hasn't yet started.
            if (deltaSecs > 0) {
                pauser.stop();
                pauser.startSingle(deltaSecs);
                return;
            }
            // We're caught up, so pause and tell the sim what our new time is so that it can
            // continue.
            SimHooks.pauseTiming();
            robotTimeSecSim.set(robotTimeSec);
        });

        final var isReadyFuture = new CompletableFuture<Boolean>();

        timeSynchronizerSim.registerValueChangedCallback(simTimeSecSim,
                new SimValueCallback() {
                    @Override
                    public synchronized void callback(String name, int handle,
                            int direction, HALValue value) {
                        double simTimeSec = value.getDouble();
                        double robotTimeSec = robotTime.get();

                        // Ignore the default initial value
                        if (simTimeSec == -1.0) {
                            return;
                        }
                        // If we asked for the simulation to start and it has started, say that
                        // we're ready.
                        if (robotTimeSecSim.get() == START_SIMULATION) {
                            if (simTimeSec == START_SIMULATION) {
                                isReadyFuture.complete(true);
                                robotTimeSecSim.set(robotTimeSec);
                            }
                            return;
                        }
                        // Otherwise, ignore notifications that the sim has started.
                        if (simTimeSec == START_SIMULATION) {
                            return;
                        }

                        // If we're not behind the sim time, there is nothing to do.
                        double deltaSecs = simTimeSec - robotTimeSec;
                        if (deltaSecs < 0.0) {
                            return;
                        }

                        // We are behind the sim time, so run until we've caught up.
                        // We use a Notifier instead of SimHooks.stepTiming() because
                        // using SimHooks.stepTiming() causes accesses to sim data to block.
                        pauser.stop();
                        pauser.startSingle(deltaSecs);
                        SimHooks.resumeTiming();
                    }
                }, true);

        // Reset the clock. Without this, *Periodic calls that should have
        // occurred while we waited, will be considered behind schedule and
        // will all happen at once.
        SimHooks.restartTiming();

        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        // Tell sim to start
        robotTimeSecSim.set(START_SIMULATION);

        // Wait up to 15 minutes for Webots to respond. On GitHub's MacOS Continuous
        // Integration servers, it can take over 8 minutes for Webots to start.
        var startedWaitingTimeMs = System.currentTimeMillis();
        var isReady = false;
        String userReminder =
                "Waiting for Webots to be ready. Please open %s in Webots."
                        .formatted(worldFile);
        System.err.println(userReminder);
        while (!isReady
                && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
            try {
                long elapsedTime =
                        System.currentTimeMillis() - startedWaitingTimeMs;
                long remainingTime = 900000 - elapsedTime;
                if (remainingTime > 0) {
                    isReady = isReadyFuture.get(remainingTime,
                            TimeUnit.MILLISECONDS);
                } else
                    break;
            } catch (TimeoutException ex) {
                System.err.println(userReminder);
            } catch (InterruptedException | ExecutionException e) {
                throw new RuntimeException(
                        "Error while waiting for Webots to be ready", e);
            }
        }
        if (!isReady) {
            throw new TimeoutException("Webots not ready in time");
        }
    }

    public void close() {
        pauser.close();
    }
}
