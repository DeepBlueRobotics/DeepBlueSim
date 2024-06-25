package org.carlmontrobotics.libdeepbluesim;

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import org.carlmontrobotics.libdeepbluesim.internal.NTConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Simulator for a WPILib TimedRobot in a Webots simulated world. Keeps the time in Webots
 * synchronized to the robot time and provides ways (eg. atSec()) to access the simulation as it
 * runs. After constructing an instance of this class, call methods other than run() to configure
 * what should happen during the simulation run, and then call run() with the TimedRobot you want to
 * simulate. The various method calls return "this" so that they can be chained.
 *
 * The Webots world must contain a Webots Robot that uses DeepBlueSim as its controller and contains
 * DeepBlueSim devices that correspond to the WPILib TimedRobot that will be passed to the run()
 * method.
 */
public class WebotsSimulator implements AutoCloseable {

    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the logging properties file specified by the
    // java.util.logging.config.file system property so that both ".level=FINE" and
    // "java.util.logging.ConsoleHandler.level=FINE". For tests via Gradle, the
    // java.util.logging.config.file system property can be configured using the systemProperty of
    // the test task.
    private static final Logger LOG =
            System.getLogger(WebotsSimulator.class.getName());

    private final Timer robotTime = new Timer();
    private final NetworkTableInstance inst;
    private final NetworkTable coordinator;
    private final StringPublisher reloadRequestPublisher;
    private final StringSubscriber reloadStatusSubscriber;
    private final DoublePublisher robotTimeSecPublisher;
    private final DoubleSubscriber simTimeSecSubscriber;
    private final StringPublisher simModePublisher;

    // We'll use this to run all NT listener callbacks sequentially on a single separate thread.
    private final ExecutorService listenerCallbackExecutor =
            Executors.newSingleThreadExecutor();

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
     * Constructs an instance that when run() is called, will cause Webots to load and, if
     * necessary, ask the user to start the specified world file.
     *
     * @param worldFilePath the path to the world file that the user should be prompted to load and
     *        start.
     * @throws FileNotFoundException if the world file does not exist
     */
    public WebotsSimulator(String worldFilePath) throws FileNotFoundException {
        withWorld(worldFilePath);
        inst = NetworkTableInstance.getDefault();
        if (ntLogLevel > 0)
            inst.addLogger(ntLogLevel, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                if (LOG.isLoggable(Level.DEBUG))
                    LOG.log(Level.DEBUG,
                            "NT instance log level %d message: %s(%d): %s"
                                .formatted(event.logMessage.level,
                                        event.logMessage.filename,
                                        event.logMessage.line,
                                        event.logMessage.message));
            });

        coordinator = inst.getTable(NTConstants.COORDINATOR_TABLE_NAME);
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                PubSubOption.keepDuplicates(true), // including duplicates
                PubSubOption.periodic(Double.MIN_VALUE), // ASAP
        };
        var reloadRequestTopic = coordinator
                .getStringTopic(NTConstants.RELOAD_REQUEST_TOPIC_NAME);
        reloadRequestPublisher = reloadRequestTopic.publish(pubSubOptions);
        reloadRequestTopic.setCached(false);

        var reloadStatusTopic = coordinator
                .getStringTopic(NTConstants.RELOAD_STATUS_TOPIC_NAME);
        reloadStatusSubscriber = reloadStatusTopic.subscribe("", pubSubOptions);

        var robotTimeSecTopic = coordinator
                .getDoubleTopic(NTConstants.ROBOT_TIME_SEC_TOPIC_NAME);
        robotTimeSecPublisher = robotTimeSecTopic.publish(pubSubOptions);
        robotTimeSecTopic.setCached(false);

        var simTimeSecTopic =
                coordinator.getDoubleTopic(NTConstants.SIM_TIME_SEC_TOPIC_NAME);
        simTimeSecSubscriber = simTimeSecTopic.subscribe(-1.0, pubSubOptions);

        var simModeTopic =
                coordinator.getStringTopic(NTConstants.SIM_MODE_TOPIC_NAME);
        simModePublisher = simModeTopic.publish(pubSubOptions);
        simModeTopic.setCached(false);
    }

    private volatile double simTimeSec = 0.0;

    private volatile int reloadCount = 0;

    /**
     * Load and, if necessary, ask the user to start the specified world file when the simulation is
     * run.
     * 
     * @param worldFilePath the path to the world file that the user should be prompted to load and
     *        start.
     * @return this object for chaining.
     * @throws FileNotFoundException if the world file does not exist
     */
    private WebotsSimulator withWorld(String worldFilePath)
            throws FileNotFoundException {
        var worldFile = new File(worldFilePath);
        if (!worldFile.isFile()) {
            throw new FileNotFoundException(worldFilePath);
        }
        onRobotInited(() -> {
            try {
                waitForUserToStart(worldFile.getAbsolutePath());
            } catch (Exception ex) {
                throw new RuntimeException(ex);
            }
        });
        return this;
    }

    private volatile boolean isReady = false;
    private volatile boolean isRobotCodeRunning = false;
    private AtomicBoolean isRobotTimeStarted = new AtomicBoolean(false);
    private boolean useStepTiming = true;

    /** Start the robot timing if we haven't already. */
    private void ensureRobotTimingStarted() {
        if (isRobotTimeStarted.getAndSet(true)) {
            return;
        }
        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        // Restart robot time.
        robotTime.stop();
        robotTime.reset();
        robotTime.start();

        var mode = useStepTiming ? NTConstants.SIM_MODE_FAST_VALUE
                : NTConstants.SIM_MODE_REALTIME_VALUE;
        LOG.log(Level.DEBUG, "Sending simMode = " + mode);
        simModePublisher.set(mode);

        var robotTimeSec = robotTime.get();
        LOG.log(Level.DEBUG, "Sending initial robotTimeSec = " + robotTimeSec);
        robotTimeSecPublisher.set(robotTimeSec);
        inst.flush();

        LOG.log(Level.INFO, "Robot is running.");
    }

    private volatile long isReadyTimestamp = Long.MAX_VALUE;

    /**
     * Load a particular world file and, if necessary, wait for the user to start it.
     * 
     * @param worldFileAbsPath the world file's absolute path to be displayed to the user.
     * @throws TimeoutException if the world hasn't been started in time.
     * 
     * @return this object for chaining
     */
    private WebotsSimulator waitForUserToStart(String worldFileAbsPath)
            throws TimeoutException {

        // Pause the clock while we wait.
        SimHooks.pauseTiming();

        isReady = false;
        isReadyTimestamp = Long.MAX_VALUE;

        // Tell the user to load the world file.
        String userReminder =
                "Waiting for Webots to be ready. Please start %s in Webots."
                        .formatted(worldFileAbsPath);

        final var isReadyFuture = new CompletableFuture<Boolean>();

        // Stop the server to disconnect any existing clients.
        inst.stopServer();

        reloadCount = 0;
        // When DeepBlueSim says that reload has completed, if it's the first time then request a
        // reload, otherwise we're ready.
        inst.addListener(reloadStatusSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final var eventValue = event.valueData.value.getString();
                    final var eventTimeStamp = event.valueData.value.getTime();
                    listenerCallbackExecutor.execute(() -> {
                        LOG.log(Level.DEBUG, "In listener, reloadStatus = %s"
                                .formatted(eventValue));
                        if (!eventValue.equals(
                                NTConstants.RELOAD_STATUS_COMPLETED_VALUE))
                            return;
                        if (reloadCount++ == 0) {
                            reloadRequestPublisher.set(worldFileAbsPath);
                            inst.flush();
                        } else {
                            isReady = true;
                            isReadyTimestamp = eventTimeStamp;
                            isReadyFuture.complete(true);
                        }
                    });
                });
        pauser.setCallback(() -> {
            LOG.log(Level.DEBUG, "In pauser callback");
            double robotTimeSec = robotTime.get();
            double deltaSecs = simTimeSec - robotTimeSec;
            // If we still haven't caught up to the simulator, then wait longer.
            // This would typically happen when robot time hasn't yet started.
            if (LOG.isLoggable(Level.DEBUG))
                LOG.log(Level.DEBUG, "deltaSecs = %g".formatted(deltaSecs));
            if (deltaSecs > 0) {
                pauser.stop();
                pauser.startSingle(deltaSecs);
                return;
            }
            // We're caught up, so pause and tell the sim what our new time is so that it can
            // continue.
            SimHooks.pauseTiming();
            if (LOG.isLoggable(Level.DEBUG))
                LOG.log(Level.DEBUG, "Sending robotTimeSec = " + robotTimeSec);
            robotTimeSecPublisher.set(robotTimeSec);
            inst.flush();
        });

        inst.addListener(simTimeSecSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final var eventTime = event.valueData.value.getTime();
                    final var eventValue = event.valueData.value.getDouble();
                    listenerCallbackExecutor.execute(() -> {
                        // Ignore values sent before simulator said it was ready.

                        // TODO: Make the reloadStatus topic into a struct topic containing both the
                        // Completed status and the initial simTimeSecs to avoid the remote
                        // possibility that the server time offset changes between when reloadStatus
                        // and simTimeSec is sent. See
                        // https://github.com/wpilibsuite/allwpilib/discussions/6712#discussioncomment-9714930
                        if (eventTime < isReadyTimestamp) {
                            LOG.log(Level.DEBUG,
                                    "Ignoring simTimeSec because it was sent before simulator said it was ready.");
                            return;
                        }

                        // Start the robot timing if we haven't already
                        ensureRobotTimingStarted();

                        // Do nothing if the robot program is not rurnning
                        if (!isRobotCodeRunning) {
                            LOG.log(Level.DEBUG,
                                    "Ignoring simTimeSec because robot code is not running.");
                            return;
                        }
                        simTimeSec = eventValue;
                        double robotTimeSec = robotTime.get();
                        if (LOG.isLoggable(Level.DEBUG))
                            LOG.log(Level.DEBUG,
                                    "Received simTimeSec of %g when robotTimeSec is %g "
                                            .formatted(simTimeSec,
                                                    robotTimeSec));

                        runAllCallbacks();

                        // If we're not behind the sim time, there is nothing else to do.
                        double deltaSecs = simTimeSec - robotTimeSec;
                        if (deltaSecs < 0.0) {
                            return;
                        }

                        if (useStepTiming) {
                            LOG.log(Level.DEBUG,
                                    "Calling SimHooks.stepTiming()");
                            SimHooks.stepTiming(deltaSecs);
                            LOG.log(Level.DEBUG,
                                    "SimHooks.stepTiming() returned");
                            // Due to rounding error, robot.getTime() might actually returns
                            // something
                            // slightly less than simTimeSec. We lie and say that we've actually
                            // caught
                            // up so that the simulator will take another step.
                            robotTimeSec = simTimeSec;
                            if (LOG.isLoggable(Level.DEBUG))
                                LOG.log(Level.DEBUG, "Sending robotTimeSec = "
                                        + robotTimeSec);
                            robotTimeSecPublisher.set(robotTimeSec);
                            inst.flush();
                            LOG.log(Level.DEBUG,
                                    "Returning from simTimeSec listener");
                            return;
                        }
                        // We are behind the sim time, so run until we've caught up.
                        // We use a Notifier instead of SimHooks.stepTiming() because
                        // using SimHooks.stepTiming() causes accesses to sim data to block.
                        pauser.stop();
                        pauser.startSingle(deltaSecs);
                        LOG.log(Level.DEBUG, "Calling SimHooks.resumeTiming()");
                        SimHooks.resumeTiming();
                        LOG.log(Level.DEBUG,
                                "SimHooks.resumeTiming() returned");
                    });
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
                LOG.log(Level.ERROR, userReminder);
            } catch (InterruptedException | ExecutionException e) {
                throw new RuntimeException(
                        "Error while waiting for Webots to be ready", e);
            }
        }
        if (!isReady) {
            throw new TimeoutException("Webots not ready in time");
        }
        LOG.log(Level.INFO, "Webots has started.");

        return this;
    }

    private void runAllCallbacks() {
        try {
            // Run all the callbacks up to the new sim time.
            everyStepCallbacks.forEach((escb) -> {
                try (var simState = new SimulationState()) {
                    escb.accept(simState);
                }
            });
            LOG.log(Level.DEBUG, "Handling atSec() callbacks");
            var oscb = oneShotCallbacks.poll();
            while (oscb != null) {
                if (LOG.isLoggable(Level.DEBUG))
                    LOG.log(Level.DEBUG,
                            "Checking callback with time = %g at simTimeSecs = %g"
                                    .formatted(oscb.timeSecs, simTimeSec));
                if (oscb.timeSecs > simTimeSec) {
                    oneShotCallbacks.add(oscb); // Put it back
                    break;
                }
                var cb = oscb.callback();
                if (cb == null)
                    continue;
                try (var simState = new SimulationState()) {
                    LOG.log(Level.DEBUG, "Calling callback");
                    oscb.callback().accept(simState);
                    LOG.log(Level.DEBUG, "Callback returned");
                }
                oscb = oneShotCallbacks.poll();
            }
        } catch (Throwable throwable) {
            // One of the callbacks threw an error, so remember it and then stop the run by clearing
            // the callbacks so that the endNotifier will be scheduled to run immediately in the
            // finally clause below.
            runExitThrowable = throwable;
            oneShotCallbacks.clear();
            everyStepCallbacks.clear();
        } finally {
            LOG.log(Level.DEBUG, "Done handling atSec() callbacks");
            if (oneShotCallbacks.isEmpty()) {
                LOG.log(Level.DEBUG,
                        "Ran last atSec() callback so scheduling end of run.");
                endNotifier.startSingle(0.0);
                LOG.log(Level.DEBUG, "Done scheduling end of run.");
            }
        }

    }

    private AtomicBoolean wereRobotInitedCallbacksRun =
            new AtomicBoolean(false);

    private void runRobotInitedCallbacksOnce() {
        if (wereRobotInitedCallbacksRun.getAndSet(true)) {
            return;
        }
        robotInitedCallbacks.forEach((callback) -> callback.run());
    }

    private final ArrayList<Runnable> robotInitedCallbacks = new ArrayList<>();

    private void onRobotInited(Runnable callback) {
        robotInitedCallbacks.add(callback);
    }

    /**
     * The current state of the simulation.
     */
    public static class SimulationState implements AutoCloseable {

        /**
         * Simulates enabling the robot in autonomous from the driver's station.
         */
        public void enableAutonomous() {
            LOG.log(Level.INFO, "Enabling in autonomous.");
            // Simulate starting autonomous
            DriverStationSim.setAutonomous(true);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }

        /**
         * Simulates enabling the robot in teleop from the driver's station.
         */
        public void enableTeleop() {
            LOG.log(Level.INFO, "Enabling in teleop.");
            // Simulate starting teleop
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }

        /**
         * Simulates disabling the robot from the driver's station.
         */
        public void disable() {
            LOG.log(Level.INFO, "Disabling");
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
        }

        /**
         * Gets the position of a specific node in the simulated world.
         * 
         * @param defPath the DEF path to the Webots node to get the position of.
         * @return the world coordinates of the requested node.
         */
        public Translation3d position(String defPath) {
            // ntTransientLogLevel = LogMessage.kDebug4;
            try {
                var watcher = Watcher.getByDefPath(defPath);
                return watcher.getPosition();
            } finally {
                ntTransientLogLevel = LogMessage.kInfo;
            }
        }

        /**
         * Gets the velocity of a specific node in the simulated world.
         * 
         * @param defPath the DEF path to the Webots node to get the position of.
         * @return the velocity of the requested node (in world coordinates)
         */
        public Translation3d velocity(String defPath) {
            // ntTransientLogLevel = LogMessage.kDebug4;
            try {
                var watcher = Watcher.getByDefPath(defPath);
                return watcher.getVelocity();
            } finally {
                ntTransientLogLevel = LogMessage.kInfo;
            }
        }

        /**
         * Gets the rotation of a specific node in the simulated world.
         * 
         * @param defPath the DEF path to the Webots node to get the position of.
         * @return the rotation of the requested node.
         */
        public Rotation3d rotation(String defPath) {
            // ntTransientLogLevel = LogMessage.kDebug4;
            try {
                var watcher = Watcher.getByDefPath(defPath);
                return watcher.getRotation();
            } finally {
                ntTransientLogLevel = LogMessage.kInfo;
            }
        }

        /**
         * Gets the angular velocity of a specific node in the simulated world.
         * 
         * @param defPath the DEF path to the Webots node to get the position of.
         * @return the angular velocity of the requested node.
         */
        public Rotation3d angularVelocity(String defPath) {
            // ntTransientLogLevel = LogMessage.kDebug4;
            try {
                var watcher = Watcher.getByDefPath(defPath);
                return watcher.getAngularVelocity();
            } finally {
                ntTransientLogLevel = LogMessage.kInfo;
            }
        }

        @Override
        public void close() {}
    }

    private record OneShotCallback(double timeSecs,
            Consumer<SimulationState> callback)
            implements Comparable<OneShotCallback> {

        @Override
        public int compareTo(OneShotCallback o) {
            return (int) Math.signum(timeSecs - o.timeSecs);
        }
    }

    private PriorityBlockingQueue<OneShotCallback> oneShotCallbacks =
            new PriorityBlockingQueue<>();

    /**
     * Register a callback to be called at a particular time during the simulation.
     * 
     * @param timeSecs the time (in seconds) at which the callback should be called
     * @param simulationStateConsumer the callback to call. It will be passed a SimulationState
     *        object.
     * 
     * @return this object for chaining
     */
    public WebotsSimulator atSec(double timeSecs,
            Consumer<SimulationState> simulationStateConsumer) {
        oneShotCallbacks
                .add(new OneShotCallback(timeSecs, simulationStateConsumer));
        return this;
    }

    private List<Consumer<SimulationState>> everyStepCallbacks =
            new ArrayList<>();


    /**
     * Register a callback to be called at every step of the simulation.
     * 
     * @param simulationStateConsumer the callback to call. It will be passed a SimulationState
     *        object.
     * 
     * @return this object for chaining
     */
    public WebotsSimulator everyStep(
            Consumer<SimulationState> simulationStateConsumer) {
        everyStepCallbacks.add(simulationStateConsumer);
        return this;
    }

    Throwable runExitThrowable = null;
    /**
     * Runs the simulation of using the specified WPILib robot.
     * 
     * @param robot the WPILib robot to run in the simulator
     * 
     */
    public void run(TimedRobot robot) {
        runExitThrowable = null;
        if (oneShotCallbacks.isEmpty()) {
            LOG.log(Level.WARNING,
                    "WebotsSimulator.atSec() was never called so planning to enable in autonomous for 15 seconds.");
            atSec(0.0, s -> {
                s.enableAutonomous();
            });
            atSec(15.0, s -> {
                s.disable();
            });
        }

        // Run the onInited callbacks once.
        // Note: the offset is -period so they are run before other WPILib periodic methods
        robot.addPeriodic(this::runRobotInitedCallbacksOnce, robot.getPeriod(),
                -robot.getPeriod());

        try (var endNotifier = new Notifier(() -> {
            LOG.log(Level.DEBUG, "Calling robot.endCompetition()");
            robot.endCompetition();
        })) {
            this.endNotifier = endNotifier;
            // HAL must be initialized or SimDeviceSim.resetData() will crash and SmartDashboard
            // might not work.
            HAL.initialize(500, 0);
            SimDeviceSim.resetData();
            SimHooks.restartTiming();
            SimHooks.resumeTiming();
            isRobotCodeRunning = true;
            robot.startCompetition();
            if (runExitThrowable != null) {
                throw new RuntimeException(runExitThrowable);
            }
        } finally {
            LOG.log(Level.DEBUG, "startCompetition() returned");
            isRobotCodeRunning = false;
            SimDeviceSim.resetData();
            // HAL.shutdown();
            LOG.log(Level.DEBUG, "run() returning");
        }
    }

    private Notifier endNotifier = null;

    /**
     * Closes this instance, freeing any resources that in holds.
     */
    public void close() {
        LOG.log(Level.DEBUG, "Closing WebotsSimulator");
        reloadRequestPublisher.close();
        reloadStatusSubscriber.close();
        robotTimeSecPublisher.close();
        simTimeSecSubscriber.close();
        Watcher.closeAll();
        inst.close();
        inst.stopServer();
        pauser.close();
        LOG.log(Level.DEBUG, "Done closing WebotsSimulator");
    }
}
