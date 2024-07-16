package org.carlmontrobotics.deepbluesim;

import java.io.File;
import java.io.IOException;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.TimeUnit;

import org.carlmontrobotics.libdeepbluesim.internal.NTConstants;
import org.carlmontrobotics.wpiws.connection.ConnectionProcessor;
import org.carlmontrobotics.wpiws.devices.SimDeviceSim;
import org.ejml.simple.SimpleMatrix;
import org.java_websocket.exceptions.WebsocketNotConnectedException;

import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.Topic;

/**
 * Synchronizes the Webots and WPILib simulations and publishes the robot's position, orientation,
 * and velocity to Network Tables
 */
public final class WebotsSupervisor {
    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the the
    // Webots/controllers/DeepBlueSim/logging.properties file so that ".level=FINE".
    private static Logger LOG =
            System.getLogger(WebotsSupervisor.class.getName());

    private static NetworkTableInstance inst = null;
    private static StringPublisher reloadStatusPublisher = null;

    private static SimDeviceSim timeSyncDeviceSim = null;

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static final int NT_LOG_LEVEL = LogMessage.kInfo;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

    private static volatile boolean isWorldLoading = false;

    final static PubSubOption[] pubSubOptions =
            new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                    PubSubOption.keepDuplicates(true), // including duplicates
                    PubSubOption.periodic(Double.MIN_VALUE), // ASAP
            };


    private static final BlockingDeque<Runnable> queuedEvents =
            new LinkedBlockingDeque<>();

    private static int usersSimulationSpeed = 0;

    // Remember the current simulation speed (default to real time if paused)
    private static void updateUsersSimulationSpeed(Supervisor robot) {
        usersSimulationSpeed =
                robot.simulationGetMode() == Supervisor.SIMULATION_MODE_PAUSE
                        ? Supervisor.SIMULATION_MODE_REAL_TIME
                        : robot.simulationGetMode();
    }

    private static CompletableFuture<Boolean> isDoneFuture =
            new CompletableFuture<>();

    private static Set<String> defPathsToPublish =
            new ConcurrentSkipListSet<String>();

    private static boolean gotSimMode;

    private static Timer delayer = null;

    /**
     * Initializes the supervisor by creating the necessary NT tables and SimDevices. This should be
     * called before connecting to the robot code.
     *
     * @param robot The robot to report on
     * @param basicTimeStep The timestep to pass to {@link Supervisor#step(int)} to advance the
     *        simulation
     */
    public static void init(Supervisor robot, int basicTimeStep) {
        // Use the default NetworkTables instance to coordinate with robot code
        inst = NetworkTableInstance.getDefault();

        addNetworkTablesLogger();

        delayer = new Timer();
        // Set to something like 100 if you are trying to reproduce the latencies we sometimes see
        // on some underpowered CI machines
        double delayMs = 0;
        ConnectionProcessor.setThreadExecutor((ev) -> {
            if (delayMs == 0) {
                queuedEvents.add(ev);
            } else {
                delayer.schedule(new TimerTask() {
                    public void run() {
                        LOG.log(Level.DEBUG, "Queueing halsim event");
                        queuedEvents.add(ev);
                    }
                }, delayMs);
            }
        });

        NetworkTable watchedNodes =
                inst.getTable(NTConstants.WATCHED_NODES_TABLE_NAME);
        var watchedNodesSubscriber = new MultiSubscriber(inst,
                new String[] {NTConstants.WATCHED_NODES_TABLE_NAME + "/"},
                pubSubOptions);
        inst.addListener(watchedNodesSubscriber, EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    queuedEvents.add(() -> {
                        var pathComponents =
                                event.valueData.getTopic().getName().split("/");
                        var name = pathComponents[pathComponents.length - 1];
                        var defPath = pathComponents[pathComponents.length - 2];
                        defPathsToPublish.add(defPath);
                        var subTable = watchedNodes.getSubTable(defPath);
                        LOG.log(Level.DEBUG, "Received request for {0} of {1}",
                                name, defPath);
                        var node = robot.getFromDef(defPath);
                        if (node == null) {
                            LOG.log(Level.ERROR,
                                    "Could not find node for the following DEF path: {0}",
                                    defPath);
                            return;
                        }
                        switch (name) {
                            case NTConstants.POSITION_TOPIC_NAME:
                                reportPositionFor(node, subTable);
                                break;
                            case NTConstants.ROTATION_TOPIC_NAME:
                                reportRotationFor(node, subTable);
                                break;
                            case NTConstants.VELOCITY_TOPIC_NAME:
                                reportVelocityFor(node, subTable);
                                break;
                            default:
                                LOG.log(Level.ERROR,
                                        "Don't know how to report ''{0}''",
                                        name);
                        }
                    });
                });

        updateUsersSimulationSpeed(robot);
        final CompletableFuture<Boolean> isDoneFuture =
                new CompletableFuture<Boolean>();

        NetworkTable coordinator =
                inst.getTable(NTConstants.COORDINATOR_TABLE_NAME);

        // Add a listener to handle reload requests.
        var reloadRequestTopic = coordinator
                .getStringTopic(NTConstants.RELOAD_REQUEST_TOPIC_NAME);
        try (var p = reloadRequestTopic.publish(pubSubOptions)) {
            reloadRequestTopic.setCached(false);
        }
        var reloadRequestSubscriber =
                reloadRequestTopic.subscribe("", pubSubOptions);
        inst.addListener(reloadRequestSubscriber,
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    var reloadRequest = event.valueData.value.getString();
                    LOG.log(Level.DEBUG, "In listener, reloadRequest = {0}",
                            reloadRequest);
                    if (reloadRequest == null)
                        return;
                    var file = new File(reloadRequest);
                    if (!file.isFile()) {
                        LOG.log(Level.ERROR,
                                "ERROR: Received a request to load file that does not exist: {0}",
                                reloadRequest);
                        return;
                    }
                    queuedEvents.add(() -> {
                        LOG.log(Level.DEBUG, "Handling reload request");
                        // Ensure we don't leave any watchers waiting for values they requested
                        // before requesting a new world.
                        closePublishers();
                        waitUntilFlushed();
                        inst.stopClient();
                        inst.close();
                        if (delayer != null) {
                            delayer.cancel();
                        }

                        LOG.log(Level.DEBUG,
                                "Updating simulation speed and mode");
                        // Unpause before reloading so that the new controller can take it's
                        // first step.
                        updateUsersSimulationSpeed(robot);
                        robot.simulationSetMode(usersSimulationSpeed);
                        isWorldLoading = true;
                        LOG.log(Level.DEBUG, "Loading world {0}",
                                reloadRequest);
                        robot.worldLoad(reloadRequest);
                        stepSimulation(robot, basicTimeStep);
                        LOG.log(Level.DEBUG, "Loaded world {0}", reloadRequest);
                    });
                });

        // Add a listener to handle simMode requests.
        var simModeTopic =
                coordinator.getStringTopic(NTConstants.SIM_MODE_TOPIC_NAME);
        try (var p = simModeTopic.publish(pubSubOptions)) {
            simModeTopic.setCached(false);
        }
        var simModeSubscriber = simModeTopic.subscribe("", pubSubOptions);
        inst.addListener(simModeSubscriber, EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    var simMode = event.valueData.value.getString();
                    LOG.log(Level.DEBUG,
                            "In listener, simMode = %s".formatted(simMode));
                    if (simMode == null)
                        return;
                    if (simMode.equals(NTConstants.SIM_MODE_FAST_VALUE)) {
                        usersSimulationSpeed = Supervisor.SIMULATION_MODE_FAST;
                    } else if (simMode
                            .equals(NTConstants.SIM_MODE_REALTIME_VALUE)) {
                        usersSimulationSpeed =
                                Supervisor.SIMULATION_MODE_REAL_TIME;
                    } else {
                        LOG.log(Level.ERROR,
                                "Unrecognized simMode of ''{0}''. Must be either 'Fast' or 'Realtime'",
                                simMode);
                    }
                    gotSimMode = true;
                    queuedEvents.add(() -> {
                        robot.simulationSetMode(usersSimulationSpeed);
                    });
                });

        // Add a listener for robotTimeSec updates that runs the sim code to catch up and notifies
        // the robot of the new sim time.
        timeSyncDeviceSim = new SimDeviceSim("DBSTimeSync");
        timeSyncDeviceSim.registerValueChangedCallback("robotTimeSec",
                (name, value) -> {
                    if (value == null) {
                        LOG.log(Level.WARNING,
                                "Ignoring null value for robotTimeSec");
                        return;
                    }
                    final double robotTimeSec = Double.parseDouble(value);
                    LOG.log(Level.DEBUG, "Received robotTimeSec={0}",
                            robotTimeSec);
                    queuedEvents.add(() -> {
                        // Keep stepping the simulation forward until the sim time is more than
                        // the robot time or the simulation ends.
                        for (;;) {
                            double simTimeSec = robot.getTime();
                            if (simTimeSec > robotTimeSec) {
                                break;
                            }
                            // Unpause if necessary
                            updateUsersSimulationSpeed(robot);
                            robot.simulationSetMode(usersSimulationSpeed);
                            boolean isDone = (stepSimulation(robot,
                                    basicTimeStep) == -1);

                            simTimeSec = robot.getTime();
                            LOG.log(Level.DEBUG, "Sending simTimeSec of {0}",
                                    simTimeSec);
                            timeSyncDeviceSim.set("simTimeSec", simTimeSec);
                            LOG.log(Level.DEBUG, "Sent simTimeSec of {0}",
                                    simTimeSec);
                            inst.flush();
                            if (isDone) {
                                isDoneFuture.complete(true);
                                break;
                            }
                            Simulation.runPeriodicMethods();
                        }
                    });
                }, true);

        // When a connection is (re-)established with the server, tell it that we are ready. When
        // the server sees that message, it knows that we will receive future messages.
        inst.addConnectionListener(true, (event) -> {
            LOG.log(Level.DEBUG, "In connection listener");
            if (event.is(Kind.kConnected)) {
                queuedEvents.add(() -> {
                    var reloadStatusTopic = coordinator.getStringTopic(
                            NTConstants.RELOAD_STATUS_TOPIC_NAME);
                    reloadStatusPublisher =
                            reloadStatusTopic.publish(pubSubOptions);
                    reloadStatusTopic.setCached(false);
                    LOG.log(Level.DEBUG, "Setting reloadStatus to Completed");
                    reloadStatusPublisher
                            .set(NTConstants.RELOAD_STATUS_COMPLETED_VALUE);
                    inst.flush();
                });
                LOG.log(Level.INFO,
                        "Connected to NetworkTables server ''{0}'' at {1}:{2,number,#}",
                        event.connInfo.remote_id, event.connInfo.remote_ip,
                        event.connInfo.remote_port);
            } else if (event.is(Kind.kDisconnected)) {
                queuedEvents.add(() -> {
                    if (isWorldLoading)
                        return;
                    LOG.log(Level.INFO,
                            "Disconnected from NetworkTables server so closing publishers and pausing simulation.");
                    closePublishers();
                    waitUntilFlushed();
                    inst.stopClient();
                    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE);

                    // The current NT implementation makes the next 4 lines unnecessary. They are
                    // here in case that implementation changes.
                    inst.close();
                    inst = NetworkTableInstance.getDefault();
                    inst.startClient4("Webots controller");
                    inst.setServer("localhost");
                });
            }
        });
    }

    @SuppressWarnings("unused")
    private static void addNetworkTablesLogger() {
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

    private static int stepSimulation(Supervisor robot, int basicTimeStep) {
        Simulation.runPeriodicMethods();
        return robot.step(basicTimeStep);
    }

    private static String lastMsg = new String();
    private static long lastMsgTimeMillis = 0,
            minTimeBetweenRepeatsMillis = 10000;

    private static void logDebugLimitingRepeats(String msg) {
        long curTimeMillis = System.currentTimeMillis();
        if (!msg.equals(lastMsg) || curTimeMillis
                - lastMsgTimeMillis >= minTimeBetweenRepeatsMillis) {
            LOG.log(Level.DEBUG,
                    "(Duplicate messages suppressed for {0} ms) {1}",
                    minTimeBetweenRepeatsMillis, msg);
            lastMsgTimeMillis = curTimeMillis;
        }
        lastMsg = msg;
    }

    /**
     * Processes messages from the robot code while advancing the simulation and running periodic
     * methods in time with the code.
     *
     * @param robot The robot to use to control the simulation
     * @param basicTimeStep The timestep to pass to {@link Supervisor#step(int)} to advance the
     *        simulation
     * @throws InterruptedException
     */
    public static void runUntilTermination(Supervisor robot, int basicTimeStep)
            throws InterruptedException, IOException {
        // Process events until simulation finishes
        while (isDoneFuture.getNow(false).booleanValue() == false) {
            try {
                // Process any pending user interface events.
                if (robot.step(0) == -1) {
                    break;
                }
                if (!queuedEvents.isEmpty()) {
                    logDebugLimitingRepeats("Processing next queued message");
                    queuedEvents.takeFirst().run();
                } else if (gotSimMode) {
                    // We are expecting a message from the robot that will tell us when to step
                    // the simulation.
                    logDebugLimitingRepeats(
                            "Waiting up to 1 second for a new message");
                    var msg = queuedEvents.pollFirst(1, TimeUnit.SECONDS);
                    if (msg == null) {
                        LOG.log(Level.WARNING,
                                "No message from robot for 1 second. It might have disconnected. Pausing.");
                        robot.simulationSetMode(
                                Supervisor.SIMULATION_MODE_PAUSE);
                        continue;
                    }
                    msg.run();
                } else if (robot
                        .simulationGetMode() != Supervisor.SIMULATION_MODE_PAUSE) {
                    // The robot code isn't going to tell us when to step the simulation and the
                    // user has unpaused it.
                    logDebugLimitingRepeats(
                            "Simulation is not paused and no robot code in control. Calling robot.step(basicTimeStep)");

                    if (stepSimulation(robot, basicTimeStep) == -1) {
                        break;
                    }
                    // LOG.log(Level.DEBUG, "robot.step(basicTimeStep) returned");
                } else {
                    // The simulation is paused and robot code isn't in control so wait a beat
                    // before checking again (so we don't suck up all the CPU)
                    logDebugLimitingRepeats(
                            "Simulation is paused and no robot code in control. Sleeping for a step");
                    Thread.sleep(basicTimeStep);
                }
            } catch (WebsocketNotConnectedException notConnectedException) {
                LOG.log(Level.WARNING,
                        "No halsim connection to the robot code. Waiting 1 second in case it is restarting. Here is the stacktrace:",
                        notConnectedException);
                Thread.sleep(1000);
            }
        }
    }

    private static void closePublishers() {
        if (reloadStatusPublisher != null)
            reloadStatusPublisher.close();
        reloadStatusPublisher = null;
        for (var entry : publisherByTopic.entrySet()) {
            entry.getValue().close();
        }
        publisherByTopic.clear();
    }

    private static void waitUntilFlushed() {
        try {
            Thread.sleep(6);
        } catch (Exception ex) {
            throw new RuntimeException(ex);
        }
        inst.flush();
    }

    private static Map<String, DoubleArrayPublisher> publisherByTopic =
            new HashMap<>();

    private static DoubleArrayPublisher getPublisherByTopic(
            DoubleArrayTopic topic) {
        var topicName = topic.getName();
        return publisherByTopic.computeIfAbsent(topicName, (tn) -> {
            return topic.publish(pubSubOptions);
        });
    }

    private static void reportVelocityFor(Node node, NetworkTable subTable) {
        var velocityTopic =
                subTable.getDoubleArrayTopic(NTConstants.VELOCITY_TOPIC_NAME);
        var publisher = getPublisherByTopic(velocityTopic);
        publisher.set(node.getVelocity().clone());
        inst.flush();
    }

    private static void reportRotationFor(Node node, NetworkTable subTable) {
        var rotationTopic =
                subTable.getDoubleArrayTopic(NTConstants.ROTATION_TOPIC_NAME);
        var nodeOrientation = node.getOrientation();
        var simpleMatrix = new SimpleMatrix(3, 3, true, nodeOrientation);
        var rotation = new Rotation3d(new Matrix<N3, N3>(simpleMatrix));
        var publisher = getPublisherByTopic(rotationTopic);
        double[] xyzArray = new double[] {rotation.getX(), rotation.getY(),
                rotation.getZ()};
        boolean hasNonFinite =
                Arrays.stream(xyzArray).anyMatch(v -> !Double.isFinite(v));
        if (hasNonFinite) {
            var topicComponents = rotationTopic.getName().split("/");
            var defPath = topicComponents[topicComponents.length - 2];
            LOG.log(Level.ERROR,
                    "rotation of {0} has at least one non-finite elements. xyzArray = {1} based on node.getOrientation() = {2}",
                    defPath, Arrays.toString(xyzArray),
                    Arrays.toString(nodeOrientation));
        }
        if (LOG.isLoggable(Level.DEBUG)) {
            LOG.log(Level.DEBUG, "Setting rotation of {0} to {1}",
                    defPathForTopic(rotationTopic), Arrays.toString(xyzArray));
        }
        publisher.set(xyzArray);
        inst.flush();
    }

    private static void reportPositionFor(Node node, NetworkTable subTable) {
        var positionTopic =
                subTable.getDoubleArrayTopic(NTConstants.POSITION_TOPIC_NAME);
        var publisher = getPublisherByTopic(positionTopic);
        double[] pos = node.getPosition().clone();
        if (LOG.isLoggable(Level.DEBUG)) {
            LOG.log(Level.DEBUG, "Setting position of {0} to {1}",
                    defPathForTopic(positionTopic), Arrays.toString(pos));
        }
        publisher.set(pos);
        inst.flush();
    }

    private static String defPathForTopic(Topic topic) {
        var topicComponents = topic.getName().split("/");
        var defPath = topicComponents[topicComponents.length - 2];
        return defPath;
    }

    private WebotsSupervisor() {}

}
