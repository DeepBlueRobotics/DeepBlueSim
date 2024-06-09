package org.carlmontrobotics.libdeepbluesim;

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
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
public class WebotsSimulator implements AutoCloseable {

    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the logging properties file specified by the
    // java.util.logging.config.file system property so that both ".level=FINE" and
    // "java.util.logging.ConsoleHandler.level=FINE". For tests via Gradle, the
    // java.util.logging.config.file system property can be configured using the systemProperty of
    // the test task.
    private static final Logger LOG =
            System.getLogger(WebotsSimulator.class.getName());

    private final TimedRobot robot;
    private final Timer robotTime = new Timer();
    private final NetworkTableInstance inst;
    private final NetworkTable coordinator;
    private final StringPublisher reloadRequestPublisher;
    private final StringSubscriber reloadStatusSubscriber;
    private final DoublePublisher robotTimeSecPublisher;
    private final DoubleSubscriber simTimeSecSubscriber;
    private final StringPublisher simModePublisher;

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
    public WebotsSimulator(TimedRobot robot) {
        this.robot = robot;
        inst = NetworkTableInstance.getDefault();
        if (ntLogLevel > 0)
            inst.addLogger(ntLogLevel, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                LOG.log(Level.DEBUG,
                        "NT instance log level %d message: %s(%d): %s"
                                .formatted(event.logMessage.level,
                                        event.logMessage.filename,
                                        event.logMessage.line,
                                        event.logMessage.message));
            });

        coordinator = inst.getTable("/DeepBlueSim/Coordinator");
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                PubSubOption.keepDuplicates(true), // including duplicates
                PubSubOption.periodic(Double.MIN_VALUE), // ASAP
        };
        var reloadRequestTopic = coordinator.getStringTopic("reloadRequest");
        reloadRequestPublisher = reloadRequestTopic.publish(pubSubOptions);
        reloadRequestTopic.setCached(false);

        var reloadStatusTopic = coordinator.getStringTopic("reloadStatus");
        reloadStatusSubscriber = reloadStatusTopic.subscribe("", pubSubOptions);

        var robotTimeSecTopic = coordinator.getDoubleTopic("robotTimeSec");
        robotTimeSecPublisher = robotTimeSecTopic.publish(pubSubOptions);
        robotTimeSecTopic.setCached(false);

        var simTimeSecTopic = coordinator.getDoubleTopic("simTimeSec");
        simTimeSecSubscriber = simTimeSecTopic.subscribe(-1.0, pubSubOptions);

        var simModeTopic = coordinator.getStringTopic("simMode");
        simModePublisher = simModeTopic.publish(pubSubOptions);
        simModeTopic.setCached(false);


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
     * Load and, if necessary, ask the user to run the specified world file when the simulation is
     * run.
     * 
     * @param worldFilePath the path to the world file that the user should be prompted to load and
     *        run.
     * @return this object for chaining.
     * @throws FileNotFoundException if the world file does not exist
     */
    public WebotsSimulator withWorld(String worldFilePath)
            throws FileNotFoundException {
        var worldFile = new File(worldFilePath);
        if (!worldFile.isFile()) {
            throw new FileNotFoundException(worldFilePath);
        }
        onRobotInited(() -> {
            try {
                waitForUserToStart(worldFile.getAbsolutePath());
                runSimulationReadyCallbacks();
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

        var mode = useStepTiming ? "Fast" : "Realtime";
        LOG.log(Level.DEBUG, "Sending simMode = " + mode);
        simModePublisher.set(useStepTiming ? "Fast" : "Realtime");

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
                    final String reloadStatus =
                            event.valueData.value.getString();
                    LOG.log(Level.DEBUG, "In listener, reloadStatus = %s"
                            .formatted(reloadStatus));
                    if (!reloadStatus.equals("Completed"))
                        return;
                    if (reloadCount++ == 0) {
                        reloadRequestPublisher.set(worldFileAbsPath);
                        inst.flush();
                    } else {
                        isReady = true;
                        isReadyTimestamp = event.valueData.value.getTime();
                        isReadyFuture.complete(true);
                    }
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
                    // Ignore values sent before simulator said it was ready.

                    // TODO: Make the reloadStatus topic into a struct topic containing both the
                    // Completed status and the initial simTimeSecs to avoid the remote possibility
                    // that the server time offset changes between when reloadStatus and simTimeSec
                    // is sent. See
                    // https://github.com/wpilibsuite/allwpilib/discussions/6712#discussioncomment-9714930
                    if (event.valueData.value.getTime() < isReadyTimestamp) {
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
                    simTimeSec = event.valueData.value.getDouble();
                    double robotTimeSec = robotTime.get();
                    if (LOG.isLoggable(Level.DEBUG))
                        LOG.log(Level.DEBUG,
                                "Received simTimeSec of %g when robotTimeSec is %g "
                                        .formatted(simTimeSec, robotTimeSec));
                    // If we're not behind the sim time, there is nothing to do.
                    double deltaSecs = simTimeSec - robotTimeSec;
                    if (deltaSecs < 0.0) {
                        return;
                    }

                    if (useStepTiming) {
                        LOG.log(Level.DEBUG, "Calling SimHooks.stepTiming()");
                        SimHooks.stepTiming(deltaSecs);
                        LOG.log(Level.DEBUG, "SimHooks.stepTiming() returned");
                        // Due to rounding error, robot.getTime() might actually returns something
                        // slightly less than simTimeSec. We lie and say that we've actually caught
                        // up so that the simulator will take another step.
                        robotTimeSec = simTimeSec;
                        if (LOG.isLoggable(Level.DEBUG))
                            LOG.log(Level.DEBUG,
                                    "Sending robotTimeSec = " + robotTimeSec);
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
                    LOG.log(Level.DEBUG, "SimHooks.resumeTiming() returned");
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

    private final ArrayList<Runnable> simulationReadyCallbacks =
            new ArrayList<>();

    private void onSimulationReady(Runnable callback) {
        simulationReadyCallbacks.add(callback);
    }

    private void runSimulationReadyCallbacks() {
        simulationReadyCallbacks.forEach((callback) -> callback.run());
    }


    /**
     * Runs a simulation of the robot enabled in autonomous for the specified amount of time.
     * 
     * @param runTime how long the simulation for run for
     * @return this object for chaining
     */
    public WebotsSimulator runAutonomous(Measure<Time> runTime) {
        LOG.log(Level.INFO, "Enabling in autonomous.");
        // Simulate starting autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        try (Notifier endNotifier = new Notifier(() -> {
            LOG.log(Level.DEBUG, "Calling robot.endCompetition()");
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
            LOG.log(Level.DEBUG, "startCompetition() returned");
            isRobotCodeRunning = false;
            SimDeviceSim.resetData();
            // HAL.shutdown();
        }
        LOG.log(Level.DEBUG, "runAutonomous() returning");
        return this;
    }

    /**
     * Passes the world position of a simulated node to a consuming functional.
     * 
     * @param defPath the DEF path of the simulated node of interest
     * @param acceptor the consuming functional
     * @return this object for chaining
     */
    public WebotsSimulator withNodePosition(String defPath,
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
        LOG.log(Level.DEBUG, "Closing WebotsManager");
        reloadRequestPublisher.close();
        reloadStatusSubscriber.close();
        robotTimeSecPublisher.close();
        simTimeSecSubscriber.close();
        watchers.forEach(w -> w.close());
        watchers.clear();
        inst.close();
        inst.stopServer();
        pauser.close();
        LOG.log(Level.DEBUG, "Done closing WebotsManager");
    }
}
