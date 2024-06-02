package org.carlmontrobotics.libdeepbluesim;

import java.util.EnumSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * A manager for Webots robots that use the DeepBlueSim controller. It keeps the time in the
 * simulator synchronized to the robot time and provides a way to add Watchers to track the
 * kinematics of Webots nodes.
 */
public class WebotsManager implements AutoCloseable {
    private final TimedRobot robot;
    private final Timer robotTime = new Timer();
    private final NetworkTableInstance inst;
    private final NetworkTable timeSync;
    private final StringEntry reloadRequestEntry;
    private final DoubleEntry robotTimeSecEntry;
    private final DoubleSubscriber simTimeSecSubscriber;

    /**
     * Time value set by the simulator and the robot to indicate that the simulation should start.
     */
    private static final double START_SIMULATION = -2.0;

    @SuppressWarnings("resource")
    private final Notifier pauser = new Notifier(() -> {
        // This is replaced in waitForUserToStart()
    });

    /**
     * Constructs an instance that connects the robot code to the Webots-simulated robot controlled
     * by DeepBlueSim.
     * 
     * @param robot the robot to connect to the Webots-simulated robot.
     */
    public WebotsManager(TimedRobot robot) {
        this.robot = robot;
        inst = NetworkTableInstance.getDefault();
        if (false)
            inst.addLogger(0, Integer.MAX_VALUE, (event) -> {
                System.out
                        .println("NT instance log level %d message: %s(%d): %s"
                                .formatted(event.logMessage.level,
                                        event.logMessage.filename,
                                        event.logMessage.line,
                                        event.logMessage.message)
                                + event.logMessage);
            });

        timeSync = inst.getTable("TimeSynchronizer");
        var pubSubOptions = new PubSubOption[] {
                // PubSubOption.sendAll(true),
                // PubSubOption.periodic(Double.MIN_VALUE)
        };
        reloadRequestEntry = timeSync.getStringTopic("reloadStatus")
                .getEntry("", PubSubOption.sendAll(true));
        robotTimeSecEntry = timeSync.getDoubleTopic("robotTimeSec")
                .getEntry(-1.0, pubSubOptions);
        simTimeSecSubscriber = timeSync.getDoubleTopic("simTimeSec")
                .subscribe(-1.0, pubSubOptions);
    }

    /**
     * Gets the current robot time.
     * 
     * @return the current robot time in seconds.
     */
    public double getTimeSecs() {
        return robotTime.get();
    }

    /**
     * Adds a Watcher for a Webots node.
     * 
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     * @return the Watcher object.
     */
    public Watcher addWatcher(String defPath) {
        return new Watcher(defPath);
    }

    /**
     * Asks the user to load and start a particular Webots world and waits until they have done so.
     * 
     * @param worldFile the world file's path to be displayed to the user.
     * @throws TimeoutException if the world hasn't been started in time.
     */
    public void waitForUserToStart(String worldFile) throws TimeoutException {
        // Tell the user to load the world file.
        String userReminder =
                "Waiting for Webots to be ready. Please open %s in Webots."
                        .formatted(worldFile);
        System.err.println(userReminder);

        // Start robot time right before the first periodic call is made so that we ignore
        // startup time.
        robotTime.stop();
        robotTime.reset();
        // Set the offset to -period to run before other WPILib periodic methods
        robot.addPeriodic(robotTime::start, robot.getPeriod(),
                -robot.getPeriod());

        // Reset the clock. Without this, *Periodic calls that should have
        // occurred while we waited, will be considered behind schedule and
        // will all happen at once.
        SimHooks.restartTiming();

        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        final var isReadyFuture = new CompletableFuture<Boolean>();

        // When the controller says it has finished reloading the world, we are ready
        timeSync.addListener("reloadStatus",
                EnumSet.of(Kind.kValueAll, Kind.kImmediate),
                (table, entry, event) -> {
                    final String reloadStatus =
                            event.valueData.value.getString();
                    System.out.println(
                            "In listener, reloadStatus = " + reloadStatus);
                    if (reloadStatus.equals("Completed")) {
                        System.out.println("Setting reloadStatus = ''");
                        reloadRequestEntry.set("",
                                reloadRequestEntry.getAtomic().serverTime
                                        + 10000);
                        inst.flush();
                        System.out.println("Done setting reloadStatus = ''");
                        isReadyFuture.complete(true);
                        robotTimeSecEntry.set(robotTime.get());
                    }
                });

        // Once the controller is running tell it to reload the world so it is in a known state. It
        // will set this value to "In Progress" before it reloads and then to "Completed" when it
        // finishes.
        reloadRequestEntry.set("Requested");
        inst.flush();

        // Wait up to 15 minutes for the controller to be ready, reminding the user every 10
        // seconds. On GitHub's MacOS Continuous Integration servers, it can take over 8 minutes for
        // Webots to start.
        var startedWaitingTimeMs = System.currentTimeMillis();
        var isReady = false;
        while (!isReady
                && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
            try {
                long elapsedTime =
                        System.currentTimeMillis() - startedWaitingTimeMs;
                long remainingTime = 900000 - elapsedTime;
                if (remainingTime > 0) {
                    isReady = isReadyFuture.get(10000, TimeUnit.MILLISECONDS);
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
        pauser.setCallback(() -> {
            double simTimeSec = simTimeSecSubscriber.get();
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
            System.out.println("Setting robotTimeSec = " + robotTimeSec);
            robotTimeSecEntry.set(robotTimeSec);
            inst.flush();
        });


        timeSync.addListener("simTimeSec",
                EnumSet.of(Kind.kValueAll, Kind.kImmediate),
                new TableEventListener() {
                    @Override
                    public synchronized void accept(NetworkTable table,
                            String key, NetworkTableEvent event) {
                        double simTimeSec = event.valueData.value.getDouble();
                        System.out.println(
                                "In listener, simTimeSec = " + simTimeSec);
                        double robotTimeSec = robotTime.get();
                        // Ignore the default initial value
                        if (simTimeSec == -1.0) {
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
                });
    }

    /**
     * Closes this instance, freeing any resources that in holds.
     */
    public void close() {
        pauser.close();
    }
}
