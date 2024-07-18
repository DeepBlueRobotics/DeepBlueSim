package org.carlmontrobotics.libdeepbluesim;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.lang.reflect.InvocationTargetException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Calendar;
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
import java.util.function.Supplier;

import org.carlmontrobotics.libdeepbluesim.internal.NTConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    // DeepBlueSim relies on the robot's NetworkTables network server and WPI's HAL sim websocket
    // server extension and only one process can be started with those at a time, because they
    // listens on TCP ports. Gradle can create multiple processes each with multiple threads each
    // with different instances of this class. To prevent conflict over use of the ports, we use a
    // file lock to ensure there is only one process has loaded this class at a time.
    private static volatile FileChannel channel = null;
    private static volatile FileLock fileLock = null;


    private final Timer robotTime = new Timer();
    private final NetworkTableInstance inst;
    private final NetworkTable coordinator;
    private final StringPublisher requestPublisher;
    private final StringSubscriber statusSubscriber;
    private final StringPublisher simModePublisher;

    // We'll use this to run all NT listener callbacks sequentially on a single separate thread.
    private final ExecutorService listenerCallbackExecutor =
            Executors.newSingleThreadExecutor();

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static final int NT_LOG_LEVEL = 0;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

    @SuppressWarnings("resource")
    private final Notifier pauser = new Notifier(() -> {
        // This is replaced in waitForUserToStart()
    });

    @SuppressWarnings("resource")
    private final Notifier robotTimeNotifier = new Notifier(() -> {
        // This is replaced in waitForUserToStart()
    });

    private Supplier<TimedRobot> robotConstructor;

    private double maxJitterSecs = 0.0;

    /**
     * @return the maximum amount of time (in seconds) that the robot code can run ahead of the
     *         simulator.
     */
    public double getMaxJitterSecs() {
        return maxJitterSecs;
    }

    /**
     * Set the maximum amount of time (in seconds) that the robot code can run ahead of the
     * simulator. Setting this to a positive value allows the robot code and simulator to run in
     * parallel (and thus faster) but with less reproducible results.
     *
     * @param maxJitterSecs the maximum amount of time (in seconds) that the robot code can run
     *        ahead of the simulator.
     * 
     * @return this object for chaining.
     */
    public WebotsSimulator setMaxJitterSecs(double maxJitterSecs) {
        this.maxJitterSecs = maxJitterSecs;
        return this;
    }

    static {
        try {
            acquireFileLock();
        } catch (InterruptedException | IOException ex) {
            LOG.log(Level.ERROR, "Unable to acquire file lock", ex);
        }
    }

    private static synchronized void acquireFileLock()
            throws InterruptedException, IOException {
        if (fileLock == null) {
            var lockFilePath = Path.of(System.getProperty("java.io.tmpdir"),
                    "WebotsSimulator.lock");
            channel = FileChannel.open(lockFilePath, StandardOpenOption.CREATE,
                    StandardOpenOption.WRITE);
            boolean hadToWait = false;
            while ((fileLock = channel.tryLock()) == null) {
                hadToWait = true;
                LOG.log(Level.WARNING,
                        "WebotsSimulator is waiting for file lock. See {0}",
                        lockFilePath);
                Thread.sleep(10000);
            }

            if (hadToWait) {
                LOG.log(Level.WARNING,
                        "WebotSimulator acquired file lock. It will be released when the process exits.");
            }
            channel.truncate(0);
            channel.write(ByteBuffer.wrap("Locked by WebotSimulator %Tc"
                    .formatted(Calendar.getInstance()).getBytes()));
            channel.force(true);
        }
    }

    private SimDouble robotTimeSecSim;
    /**
     * Constructs an instance that when run() is called, will cause Webots to load and, if
     * necessary, ask the user to start the specified world file.
     *
     * @param worldFilePath the path to the world file that the user should be prompted to load and
     *        start.
     * @param robotConstructor a function which creates an instance of the class of the TimedRobot
     *        to run. Note: The supplier should create the TimedRobot instance (e.g.
     *        <code>Robot::new</code>). Passing an existing robot object (e.g.
     *        <code>() -> robot</code>) will lead to flaky tests.
     * @throws InterruptedException if interrupted while waiting to get exclusive use of the needed
     *         TCP ports.
     * @throws IOException if an IO error occurs while attempting to get exclusive use of the needed
     *         TCP ports.
     */
    public WebotsSimulator(String worldFilePath,
            Supplier<TimedRobot> robotConstructor)
            throws InterruptedException, IOException {
        this.robotConstructor = robotConstructor;
        withWorld(worldFilePath);
        inst = NetworkTableInstance.getDefault();
        addNetworkTablesLogger();

        coordinator = inst.getTable(NTConstants.COORDINATOR_TABLE_NAME);
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                PubSubOption.keepDuplicates(true), // including duplicates
                PubSubOption.periodic(Double.MIN_VALUE), // ASAP
        };
        var requestTopic =
                coordinator.getStringTopic(NTConstants.REQUEST_TOPIC_NAME);
        requestPublisher = requestTopic.publish(pubSubOptions);
        requestTopic.setCached(false);

        var statusTopic =
                coordinator.getStringTopic(NTConstants.STATUS_TOPIC_NAME);
        statusSubscriber = statusTopic.subscribe("", pubSubOptions);

        var simModeTopic =
                coordinator.getStringTopic(NTConstants.SIM_MODE_TOPIC_NAME);
        simModePublisher = simModeTopic.publish(pubSubOptions);
        simModeTopic.setCached(false);
    }

    @SuppressWarnings("unused")
    private void addNetworkTablesLogger() {
        if (NT_LOG_LEVEL > 0) {
            inst.addLogger(NT_LOG_LEVEL, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                LOG.log(Level.DEBUG,
                        "NT instance log level {0} message: {1}({2}): {3}",
                        event.logMessage.level, event.logMessage.filename,
                        event.logMessage.line, event.logMessage.message);
            });
        }
    }

    private volatile double simTimeSec = 0.0;

    private volatile int loadCount = 0;

    private File worldFile = null;

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
        this.worldFile = worldFile;
        onRobotInited(() -> {
            try {
                startTimeSync();
            } catch (Exception ex) {
                throw new RuntimeException(ex);
            }
        });
        return this;
    }

    private volatile boolean isReady = false;
    private volatile boolean isRobotCodeRunning = false;
    private AtomicBoolean isRobotTimeStarted = new AtomicBoolean(false);

    // TODO: Add a method that allows this to be set to false if the user wants to run the *robot*
    // code in real time instead of as fast as possible. That might be useful if there is robot (or
    // vendor) code that uses a time source other than WPILib. Not temporarily making it a constant
    // (ie. final) because that will cause the compiler/vscode to warn about dead code.
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
        LOG.log(Level.DEBUG, "Sending simMode = {0}", mode);
        simModePublisher.set(mode);

        inst.flush();

        LOG.log(Level.INFO, "Robot is running.");
    }

    private CompletableFuture<Boolean> isReadyFuture = null;

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

        // Tell the user to load the world file.
        String userReminder =
                "Waiting for Webots to be ready. Please start %s in Webots."
                        .formatted(worldFileAbsPath);

        isReadyFuture = new CompletableFuture<Boolean>();

        // Stop the server to disconnect any existing clients.
        inst.stopServer();

        loadCount = 0;
        // When DeepBlueSim says that load has completed, if it's the first time then request a
        // load, otherwise we're ready.
        inst.addListener(statusSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final var eventValue = event.valueData.value.getString();
                    listenerCallbackExecutor.execute(() -> {
                        LOG.log(Level.DEBUG, "In listener, status = {0}",
                                        eventValue);
                        if (!eventValue.equals(
                                NTConstants.STATUS_COMPLETED_VALUE))
                            return;
                        if (loadCount++ == 0) {
                            var request = String.format("%s %s",
                                    NTConstants.REQUEST_LOAD_VERB,
                                    worldFileAbsPath);
                            LOG.log(Level.DEBUG, "Sending request = {0}",
                                            request);
                            requestPublisher.set(request);
                            inst.flush();
                            LOG.log(Level.DEBUG, "Sent request = {0}",
                                    request);
                        } else {
                            isReady = true;
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
            LOG.log(Level.DEBUG, "deltaSecs = {0}", deltaSecs);
            if (deltaSecs > 0) {
                pauser.stop();
                pauser.startSingle(deltaSecs);
                return;
            }
            // We're caught up, so pause and tell the sim what our new time is so that it can
            // continue.
            SimHooks.pauseTiming();
            LOG.log(Level.DEBUG, "Sending robotTimeSec = {0}", robotTimeSec);
            robotTimeSecSim.set(robotTimeSec);
        });

        robotTimeNotifier.setCallback(() -> {
            LOG.log(Level.DEBUG, "In robotTimeNotifier callback");
            // Due to rounding error, robot.getTime() might actually return something slightly less
            // than simTimeSec. We lie and say that we've actually caught up so that the simulator
            // will take another step.
            var timeSec = simTimeSec;
            LOG.log(Level.DEBUG, "Sending robotTimeSec = {0}", timeSec);
            robotTimeSecSim.set(timeSec);
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
                // Restart the NT server in case we somehow missed that the loading of the world
                // completed.
                inst.stopServer();
                inst.startServer();
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

    private void sendRobotTime() {
        robotTimeNotifier.startSingle(0);
        SimHooks.stepTiming(0);
    }

    private SimDevice timeSyncDevice;
    private SimDeviceSim timeSyncDeviceSim;

    private void startTimeSync() {
        LOG.log(Level.DEBUG, "In startTimeSync()");
        ensureRobotTimingStarted();
        LOG.log(Level.DEBUG, "Created SimDevice with name {0}",
                timeSyncDevice.getName());
        timeSyncDeviceSim = new SimDeviceSim("DBSTimeSync");
        timeSyncDeviceSim.registerValueChangedCallback(
                timeSyncDeviceSim.getDouble("simTimeSec"),
                (name, handle, direction, value) -> {
                    LOG.log(Level.DEBUG,
                            "In simTimeSec value changed callback");
                    final var eventValue = value.getDouble();
                    listenerCallbackExecutor.execute(() -> {
                        LOG.log(Level.DEBUG, "Got simTimeSec value of {0}",
                                eventValue);
                        if (eventValue < 0.0) {
                            LOG.log(Level.DEBUG,
                                    "Ignoring simTimeSec value of {0}",
                                    eventValue);
                            return;
                        }
                        simTimeSec = eventValue;

                        // Do nothing if the robot program is not rurnning
                        if (!isRobotCodeRunning) {
                            LOG.log(Level.DEBUG,
                                    "Ignoring simTimeSec because robot code is not running.");
                            return;
                        }
                        double robotTimeSec = robotTime.get();
                        LOG.log(Level.DEBUG,
                                "Received simTimeSec of {0} when robotTimeSec is {1}",
                                simTimeSec, robotTimeSec);

                        runAllCallbacks(robotTimeSec, simTimeSec);

                        // If we're not behind the sim time, there is nothing else to do.
                        double deltaSecs =
                                simTimeSec + maxJitterSecs - robotTimeSec;
                        if (deltaSecs < 0.0) {
                            return;
                        }

                        if (useStepTiming) {
                            LOG.log(Level.DEBUG,
                                    "Calling SimHooks.stepTiming({0})",
                                    deltaSecs);
                            SimHooks.stepTiming(deltaSecs);
                            LOG.log(Level.DEBUG,
                                    "SimHooks.stepTiming() returned");
                            // Send the new robot time
                            sendRobotTime();
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
                }, true);

        waitForHALSimWSConnection();

        listenerCallbackExecutor.execute(() -> {
            simTimeSec = 0.0;
            sendRobotTime();
        });
    }

    // NOTE: Until https://github.com/wpilibsuite/allwpilib/issues/6842 is fixed, this method should
    // not be called until all SimDevices have been created (both in the robot code and our own
    // "DBSTimeSync" SimDevice). Creating a SimDevice after the HALSimWS connection has been
    // established can result in a deadlock.
    private void waitForHALSimWSConnection() {
        var halSimWSConnected = new CompletableFuture<Void>();
        inst.addListener(statusSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final var eventValue = event.valueData.value.getString();
                    listenerCallbackExecutor.execute(() -> {
                        LOG.log(Level.DEBUG, "In listener, status = {0}",
                                eventValue);
                        if (!eventValue.equals(
                                NTConstants.STATUS_HALSIMWS_CONNECTED_VALUE))
                            return;
                        halSimWSConnected.complete(null);
                    });
                });

        requestPublisher.set(NTConstants.REQUEST_HALSIMWS_CONNECTION_VERB);
        inst.flush();

        try {
            halSimWSConnected.get(30000, TimeUnit.MILLISECONDS);
        } catch (TimeoutException | InterruptedException
                | ExecutionException e) {
            throw new RuntimeException(
                    "Error while waiting for HALSimWS connection", e);
        }
    }

    private void runAllCallbacks(double robotTimeSec, double simTimeSec) {
        try (var simState = new SimulationState(robotTimeSec, simTimeSec)) {
            // Run all the callbacks up to the new sim time.
            everyStepCallbacks.forEach(escb -> escb.accept(simState));
            LOG.log(Level.DEBUG, "Handling atSec() callbacks");
            OneShotCallback oscb;
            while ((oscb = oneShotCallbacks.poll()) != null) {
                LOG.log(Level.DEBUG,
                        "Checking callback with time = {0} at simTimeSec = {1}",
                                oscb.timeSecs, simTimeSec);
                if (oscb.timeSecs > simTimeSec) {
                    oneShotCallbacks.add(oscb); // Put it back
                    break;
                }
                var cb = oscb.callback();
                if (cb == null)
                    continue;
                LOG.log(Level.DEBUG, "Calling callback");
                oscb.callback().accept(simState);
                LOG.log(Level.DEBUG, "Callback returned");
            }
        } catch (Throwable throwable) {
            // One of the callbacks threw an error, so remember it and then stop the run by clearing
            // the callbacks so that the endNotifier will be scheduled to run immediately in the
            // finally clause below.
            runExitThrowable = throwable;
            oneShotCallbacks.clear();
            everyStepCallbacks.clear();
        } finally {
            LOG.log(Level.DEBUG, "Done handling callbacks");
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

        /**
         * @return the time (in seconds) since the robot code started, according to the robot code.
         */
        public double getRobotTimeSec() {
            return robotTimeSec;
        }

        /**
         * @return the time (in seconds) since the simulation started, according to the simulator.
         */
        public double getSimTimeSec() {
            return simTimeSec;
        }

        private SimulationState() {}

        private double robotTimeSec, simTimeSec;

        private SimulationState(double robotTimeSec, double simTimeSec) {
            this.robotTimeSec = robotTimeSec;
            this.simTimeSec = simTimeSec;
        }
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

    boolean endCompetitionCalled = false;

    private void endCompetition(TimedRobot robot) {
        // As of WPILib 2024.3.1, calling endCompetition() more than once after calling
        // startCompetition() doesn't seem to cause any problems, but we make sure we only call it
        // once just in case that changes in the future.
        if (!endCompetitionCalled) {
            LOG.log(Level.DEBUG, "Calling robot.endCompetition()");
            robot.endCompetition();
            endCompetitionCalled = true;
        }
    }

    /**
     * Runs the simulation.
     * 
     * @throws SecurityException if the robot class's constructor is inaccessible due to module
     *         security
     * @throws NoSuchMethodException if the robot class doesn't have a default constructor
     * @throws InvocationTargetException if the robot class's default constructor throws an
     *         exception
     * @throws IllegalAccessException if the robot class's default constructor is not accessible
     *         (e.g. not public)
     * @throws InstantiationException if the robot class is abstract
     * @throws TimeoutException if the world is not loaded running withing Webots in a reasonable
     *         amount of time
     * 
     */
    public void run() throws InstantiationException, IllegalAccessException,
            InvocationTargetException, NoSuchMethodException,
            SecurityException, TimeoutException {
        endCompetitionCalled = false;
        // HAL must be initialized or SmartDashboard might not work.
        HAL.initialize(500, 0);
        waitForUserToStart(worldFile.getAbsolutePath());

        timeSyncDevice = SimDevice.create("DBSTimeSync");
        robotTimeSecSim = timeSyncDevice.createDouble("robotTimeSec",
                SimDevice.Direction.kOutput, -1.0);
        timeSyncDevice.createDouble("simTimeSec", SimDevice.Direction.kInput,
                -1.0);

        // Restart timing before robot is constructed to ensure that timed callbacks added during
        // the constructor (and our own onRobotInit callback) work properly.
        SimHooks.restartTiming();
        SimHooks.pauseTiming();
        try (TimedRobot robot = robotConstructor.get();
                var endNotifier = new Notifier(() -> {
                    endCompetition(robot);
                })) {
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
            robot.addPeriodic(this::runRobotInitedCallbacksOnce,
                    robot.getPeriod(), -robot.getPeriod());

            this.endNotifier = endNotifier;
            SimHooks.resumeTiming();
            isRobotCodeRunning = true;
            try {
                robot.startCompetition();
            } finally {
                // Always call endCompetition() so that WPILib's notifier is stopped. If it isn't
                // future tests will hang on calls to stepTiming().
                endCompetition(robot);
            }
            if (runExitThrowable != null) {
                throw new RuntimeException(runExitThrowable);
            }
        } catch (IllegalArgumentException ex) {
            // This should not be possible.
            throw new RuntimeException(ex);
        } finally {
            LOG.log(Level.DEBUG, "startCompetition() returned");
            isRobotCodeRunning = false;
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
        requestPublisher.close();
        statusSubscriber.close();
        Watcher.closeAll();
        inst.close();
        inst.stopServer();
        pauser.close();
        if (timeSyncDevice != null) {
            timeSyncDevice.close();
        }
        try {
            listenerCallbackExecutor.shutdown();
        } catch (SecurityException ex) {
            LOG.log(Level.ERROR, "Could not shutdown callback executor.", ex);
        }

        LOG.log(Level.DEBUG, "Done closing WebotsSimulator");
    }
}
