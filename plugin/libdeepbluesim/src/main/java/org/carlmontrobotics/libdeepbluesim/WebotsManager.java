package org.carlmontrobotics.libdeepbluesim;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
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
    private final NetworkTable coordinator;
    private final StringPublisher reloadRequestPublisher;
    private final StringSubscriber reloadStatusSubscriber;
    private final DoublePublisher robotTimeSecPublisher;
    private final DoubleSubscriber simTimeSecSubscriber;

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static int ntLogLevel = 0;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

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
        if (ntLogLevel > 0)
            inst.addLogger(ntLogLevel, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                System.out
                        .println("NT instance log level %d message: %s(%d): %s"
                                .formatted(event.logMessage.level,
                                        event.logMessage.filename,
                                        event.logMessage.line,
                                        event.logMessage.message));
            });

        coordinator = inst.getTable("/DeepBlueSim/Coordinator");
        var pubSubOptions = new PubSubOption[] {
                PubSubOption.sendAll(true),
                PubSubOption.periodic(Double.MIN_VALUE)
        };
        var reloadRequestTopic = coordinator.getStringTopic("reloadRequest");
        reloadRequestPublisher = reloadRequestTopic.publish(
                PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        reloadRequestTopic.setCached(false);

        var reloadStatusTopic = coordinator.getStringTopic("reloadStatus");
        reloadStatusSubscriber =
                reloadStatusTopic.subscribe("", PubSubOption.sendAll(true));

        var robotTimeSecTopic = coordinator.getDoubleTopic("robotTimeSec");
        robotTimeSecPublisher = robotTimeSecTopic.publish(pubSubOptions);
        robotTimeSecTopic.setCached(false);

        var simTimeSecTopic = coordinator.getDoubleTopic("simTimeSec");
        simTimeSecSubscriber = simTimeSecTopic.subscribe(-1.0, pubSubOptions);

        // Run the onInited callbacks once.
        // Note: the offset is -period so they are run before other WPILib periodic methods
        robot.addPeriodic(this::runRobotInitedCallbacksOnce, robot.getPeriod(),
                -robot.getPeriod());
    }

    /**
     * Gets the current robot time.
     * 
     * @return the current robot time in seconds.
     */
    public double getTimeSecs() {
        return robotTime.get();
    }

    private Set<Watcher> watchers = new HashSet<>();
    /**
     * Adds a Watcher for a Webots node.
     * 
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     * @return the Watcher object.
     */
    public Watcher addWatcher(String defPath) {
        var w = new Watcher(defPath);
        watchers.add(w);
        return w;
    }

    private volatile double simTimeSec = 0.0;

    private volatile int reloadCount = 0;

    /**
     * Asks the user to load and run the specified world file when the simulation is run.
     * 
     * @param worldFile the path to the world file that the user should be prompted to load and run.
     * @return this object for chaining.
     */
    public WebotsManager withWorld(String worldFile) {
        onRobotInited(() -> {
            try {
                waitForUserToStart(worldFile);
                runSimulationReadyCallbacks();
            } catch (Exception ex) {
                throw new RuntimeException(ex);
            }
        });
        return this;
    }

    private volatile boolean isReady = false;
    private volatile boolean isRobotCodeRunning = false;
    private boolean isRobotTimeStarted = false;
    private boolean useStepTiming = true;

    /** Start the robot timing if we haven't already. */
    private synchronized void ensureRobotTimingStarted() {
        if (isRobotTimeStarted) {
            return;
        }
        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        // Restart robot time.
        robotTime.stop();
        robotTime.reset();
        robotTime.start();

        var robotTimeSec = robotTime.get();
        System.out.println("Sending initial robotTimeSec = " + robotTimeSec);
        robotTimeSecPublisher.set(robotTimeSec);
        inst.flush();

        System.out.println("Robot is running.");
        isRobotTimeStarted = true;
    }

    /**
     * Asks the user to load and start a particular Webots world and waits until they have done so.
     * 
     * @param worldFile the world file's path to be displayed to the user.
     * @throws TimeoutException if the world hasn't been started in time.
     * 
     * @return this object for chaining
     */
    private WebotsManager waitForUserToStart(String worldFile)
            throws TimeoutException {

        // Pause the clock while we wait.
        SimHooks.pauseTiming();

        isReady = false;

        // Tell the user to load the world file.
        String userReminder =
                "Waiting for Webots to be ready. Please open %s in Webots."
                        .formatted(worldFile);
        System.err.println(userReminder);

        final var isReadyFuture = new CompletableFuture<Boolean>();

        // Stop the server to disconnect any existing clients.
        inst.stopServer();

        reloadCount = 0;
        // When DeepBlueSim says that reload has completed, if it's the first time then request a
        // reload, otherwise we're ready.
        inst.addListener(reloadStatusSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final String reloadStatus =
                            event.valueData.value.getString();
                    System.out.println(
                            "In listener, reloadStatus = %s"
                                    .formatted(reloadStatus));
                    if (!reloadStatus.equals("Completed"))
                        return;
                    if (reloadCount++ == 0) {
                        reloadRequestPublisher.set("Requested");
                        inst.flush();
                    } else {
                        isReady = true;
                        isReadyFuture.complete(true);
                    }
                });
        pauser.setCallback(() -> {
            System.out.println("In pauser callback");
            double robotTimeSec = robotTime.get();
            double deltaSecs = simTimeSec - robotTimeSec;
            // If we still haven't caught up to the simulator, then wait longer.
            // This would typically happen when robot time hasn't yet started.
            System.out.println("deltaSecs = %g".formatted(deltaSecs));
            if (deltaSecs > 0) {
                pauser.stop();
                pauser.startSingle(deltaSecs);
                return;
            }
            // We're caught up, so pause and tell the sim what our new time is so that it can
            // continue.
            SimHooks.pauseTiming();
            System.out.println("Sending robotTimeSec = " + robotTimeSec);
            robotTimeSecPublisher.set(robotTimeSec);
            inst.flush();
        });

        inst.addListener(simTimeSecSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    // Ignore values sent before simulator said it was ready.
                    if (!isReady)
                        return;

                    // Start the robot timing if we haven't already
                    ensureRobotTimingStarted();

                    // Do nothing if the robot program is not rurnning
                    if (!isRobotCodeRunning) {
                        System.out.println(
                                "Ignoring simTimeSec because robot code is not running.");
                        return;
                    }
                    simTimeSec = event.valueData.value.getDouble();
                    double robotTimeSec = robotTime.get();
                    System.out.println(
                            "Received simTimeSec of %g when robotTimeSec is %g "
                                    .formatted(simTimeSec, robotTimeSec));
                    // If we're not behind the sim time, there is nothing to do.
                    double deltaSecs = simTimeSec - robotTimeSec;
                    if (deltaSecs < 0.0) {
                        return;
                    }

                    if (useStepTiming && simTimeSec < runTimeSecs) {
                        var timingStepped = new CompletableFuture<>();
                        try (var timingSteppedCompleter = new Notifier(() -> {
                            timingStepped.complete(null);
                        })) {
                            timingSteppedCompleter.startSingle(0);
                            System.out.println(
                                    "Calling SimHooks.stepTimingAsync()");
                            SimHooks.stepTimingAsync(deltaSecs);
                            System.out.println("Calling timingStepped.get()");
                            timingStepped.get((long) (2 * deltaSecs + 1.0),
                                    TimeUnit.SECONDS);
                            System.out.println("timingStepped.get() returned");
                        } catch (Exception ex) {
                            throw new RuntimeException(ex);
                        }
                        robotTimeSec = robotTime.get();
                        System.out.println(
                                "Sending robotTimeSec = " + robotTimeSec);
                        robotTimeSecPublisher.set(robotTimeSec);
                        inst.flush();
                        System.out
                                .println("Returning from simTimeSec listener");
                        return;
                    }
                    // We are behind the sim time, so run until we've caught up.
                    // We use a Notifier instead of SimHooks.stepTiming() because
                    // using SimHooks.stepTiming() causes accesses to sim data to block.
                    pauser.stop();
                    pauser.startSingle(deltaSecs + robot.getPeriod());
                    System.out.println("Calling SimHooks.resumeTiming()");
                    SimHooks.resumeTiming();
                    System.out.println("SimHooks.resumeTiming() returned");
                });
        // Restart the server so that the clients will reconnect.
        inst.startServer();

        // Wait up to 15 minutes for DeepBlueSim to be ready, reminding the user every 10 seconds.
        // On GitHub's MacOS Continuous Integration servers, it can take over 8 minutes for Webots
        // to start.
        var startedWaitingTimeMs = System.currentTimeMillis();
        while (!isReady
                && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
            try {
                long elapsedTime =
                        System.currentTimeMillis() - startedWaitingTimeMs;
                long remainingTime = 900000 - elapsedTime;
                if (remainingTime > 0) {
                    isReadyFuture.get(10000, TimeUnit.MILLISECONDS);
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
        System.out.println("Webots has started.");

        return this;
    }

    private boolean wereRobotInitedCallbacksRun = false;

    private synchronized void runRobotInitedCallbacksOnce() {
        if (wereRobotInitedCallbacksRun) {
            return;
        }
        robotInitedCallbacks.forEach((callback) -> callback.run());
        wereRobotInitedCallbacksRun = true;
    }

    private final ArrayList<Runnable> robotInitedCallbacks = new ArrayList<>();

    private void onRobotInited(Runnable callback) {
        robotInitedCallbacks.add(callback);
    }

    private final ArrayList<Runnable> simulationReadyCallbacks =
            new ArrayList<>();

    private void onSimulationReady(Runnable callback) {
        simulationReadyCallbacks.add(callback);
    }

    private void runSimulationReadyCallbacks() {
        simulationReadyCallbacks.forEach((callback) -> callback.run());
    }

    private double runTimeSecs = 0.0;

    /**
     * Runs a simulation of the robot enabled in autonomous for the specified amount of time.
     * 
     * @param runTime how long the simulation for run for
     * @return this object for chaining
     */
    public WebotsManager runAutonomous(Measure<Time> runTime) {
        runTimeSecs = runTime.in(Seconds);
        System.out.println("Enabling in autonomous.");
        // Simulate starting autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        try (Notifier endNotifier = new Notifier(() -> {
            System.out.println("Calling robot.endCompetition()");
            robot.endCompetition();
        })) {
            // HAL must be initialized or SimDeviceSim.resetData() will crash and SmartDashboard
            // might not work.
            HAL.initialize(500, 0);
            SimDeviceSim.resetData();
            onSimulationReady(() -> {
                endNotifier.startSingle(runTime.in(Seconds));
            });
            SimHooks.restartTiming();
            SimHooks.resumeTiming();
            isRobotCodeRunning = true;
            robot.startCompetition();
        } finally {
            System.out.println("startCompetition() returned");
            isRobotCodeRunning = false;
            SimDeviceSim.resetData();
            // HAL.shutdown();
        }
        System.out.println("runAutonomous() returning");
        return this;
    }

    /**
     * Passes the world position of a simulated node to a consuming functional.
     * 
     * @param defPath the DEF path of the simulated node of interest
     * @param acceptor the consuming functional
     * @return this object for chaining
     */
    public WebotsManager withNodePosition(String defPath,
            Consumer<Translation3d> acceptor) {
        try (var watcher = new Watcher(defPath)) {
            var pos = watcher.getPosition();
            acceptor.accept(pos);
            return this;
        }
    }
    /**
     * Closes this instance, freeing any resources that in holds.
     */
    public void close() {
        System.out.println("Closing WebotsManager");
        reloadRequestPublisher.close();
        reloadStatusSubscriber.close();
        robotTimeSecPublisher.close();
        simTimeSecSubscriber.close();
        watchers.forEach(w -> w.close());
        watchers.clear();
        inst.close();
        inst.stopServer();
        pauser.close();
        System.out.println("Done closing WebotsManager");
    }
}
