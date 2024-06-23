import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.TimeUnit;

import org.ejml.simple.SimpleMatrix;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.exceptions.WebsocketNotConnectedException;
import org.team199.deepbluesim.SimRegisterer;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.RunningObject;
import org.team199.wpiws.connection.WSConnection;

import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches
// the
// the name of the jar.
public class DeepBlueSim {
    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the the
    // Webots/controllers/DeepBlueSim/logging.properties file so that ".level=FINE".
    private static Logger LOG = null;

    private static final BlockingDeque<Runnable> queuedMessages =
            new LinkedBlockingDeque<>();

    private static RunningObject<WebSocketClient> wsConnection = null;

    private static int usersSimulationSpeed = 0;

    // Remember the current simulation speed (default to real time if paused)
    private static void updateUsersSimulationSpeed(Supervisor robot) {
        usersSimulationSpeed =
                robot.simulationGetMode() == Supervisor.SIMULATION_MODE_PAUSE
                        ? Supervisor.SIMULATION_MODE_REAL_TIME
                        : robot.simulationGetMode();
    }

    private static Set<String> defPathsToPublish =
            new ConcurrentSkipListSet<String>();

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static int ntLogLevel = LogMessage.kInfo;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

    private static volatile boolean isWorldLoading = false;

    final static PubSubOption[] pubSubOptions =
            new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                    PubSubOption.keepDuplicates(true), // including duplicates
                    PubSubOption.periodic(Double.MIN_VALUE), // ASAP
            };

    private static void startNetworkTablesClient(NetworkTableInstance inst) {
        inst.startClient4("Webots controller");
        inst.setServer("localhost");
    }

    private static NetworkTableInstance inst = null;
    private static DoublePublisher simTimeSecPublisher = null;
    private static StringPublisher reloadStatusPublisher = null;

    public static void main(String[] args) throws IOException {
        System.setProperty("java.util.logging.config.file",
                "logging.properties");
        LOG = System.getLogger(DeepBlueSim.class.getName());
        LOG.log(Level.DEBUG, "Starting log");
        // Set up exception handling to log to stderr and exit
        {
            UncaughtExceptionHandler eh = new UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread arg0, Throwable arg1) {
                    arg1.printStackTrace(System.err);
                    System.err.flush();
                    System.exit(1);
                }
            };
            Thread.setDefaultUncaughtExceptionHandler(eh);
            Thread.currentThread().setUncaughtExceptionHandler(eh);
        }

        // Boilerplate code so we can use NetworkTables
        {
            NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
            WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
            WPIMathJNI.Helper.setExtractOnStaticLoad(false);

            CombinedRuntimeLoader.loadLibraries(DeepBlueSim.class, "wpiutiljni",
                    "wpimathjni", "ntcorejni");
        }

        // Use the default NetworkTables instance to coordinate with robot code
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

        final Supervisor robot = new Supervisor();

        NetworkTable watchedNodes = inst.getTable("/DeepBlueSim/WatchedNodes");
        var multiSubscriber = new MultiSubscriber(inst,
                new String[] {"/DeepBlueSim/WatchedNodes/"}, pubSubOptions);
        inst.addListener(multiSubscriber, EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    queuedMessages.add(() -> {
                        var pathComponents =
                                event.valueData.getTopic().getName().split("/");
                        var name = pathComponents[pathComponents.length - 1];
                        var defPath = pathComponents[pathComponents.length - 2];
                        defPathsToPublish.add(defPath);
                        var subTable = watchedNodes.getSubTable(defPath);
                        LOG.log(Level.DEBUG, "Received request for %s of %s"
                                .formatted(name, defPath));
                        var node = robot.getFromDef(defPath);
                        if (node == null) {
                            LOG.log(Level.ERROR,
                                    "Could not find node for the following DEF path: "
                                            + defPath);
                            return;
                        }
                        switch (name) {
                            case "position":
                                reportPositionFor(node, subTable);
                                break;
                            case "rotation":
                                reportRotationFor(node, subTable);
                                break;
                            case "velocity":
                                reportVelocityFor(node, subTable);
                                break;
                            default:
                                LOG.log(Level.ERROR,
                                        "Don't know how to report '%s'"
                                                .formatted(name));
                        }
                    });
                });

        ConnectionProcessor.setThreadExecutor(queuedMessages::add);

        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));

        if (!robot.getSupervisor()) {
            LOG.log(Level.ERROR,
                    "The robot does not have supervisor=true. This is required to detect devices.");
            System.exit(1);
        }

        // Get the basic timestep to use for calls to robot.step()
        final int basicTimeStep = (int) Math.round(robot.getBasicTimeStep());

        Simulation.init(robot, robot.getBasicTimeStep());

        updateUsersSimulationSpeed(robot);
        final CompletableFuture<Boolean> isDoneFuture =
                new CompletableFuture<Boolean>();

        NetworkTable coordinator = inst.getTable("/DeepBlueSim/Coordinator");

        // Add a listener to handle reload requests.
        var reloadRequestTopic = coordinator.getStringTopic("reloadRequest");
        try (var p = reloadRequestTopic.publish(pubSubOptions)) {
            reloadRequestTopic.setCached(false);
        }
        var reloadRequestSubscriber =
                reloadRequestTopic.subscribe("", pubSubOptions);
        inst.addListener(reloadRequestSubscriber, EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    var reloadRequest = event.valueData.value.getString();
                    LOG.log(Level.DEBUG, "In listener, reloadRequest = %s"
                            .formatted(reloadRequest));
                    if (reloadRequest == null)
                        return;
                    var file = new File(reloadRequest);
                    if (!file.isFile()) {
                        LOG.log(Level.ERROR,
                                "ERROR: Received a request to load file that does not exist: %s"
                                        .formatted(reloadRequest));
                        return;
                    }
                    queuedMessages.add(() -> {
                        closePublishers();
                        waitUntilFlushed();
                        inst.stopClient();
                        inst.close();

                        // Unpause before reloading so that the new controller can take it's
                        // first step.
                        updateUsersSimulationSpeed(robot);
                        robot.simulationSetMode(usersSimulationSpeed);
                        isWorldLoading = true;
                        robot.worldLoad(reloadRequest);
                    });
                });

        // Add a listener to handle simMode requests.
        var simModeTopic = coordinator.getStringTopic("simMode");
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
                    if (simMode.equals("Fast")) {
                        usersSimulationSpeed = Supervisor.SIMULATION_MODE_FAST;
                    } else if (simMode.equals("Realtime")) {
                        usersSimulationSpeed =
                                Supervisor.SIMULATION_MODE_REAL_TIME;
                    } else {
                        LOG.log(Level.ERROR,
                                "Unrecognized simMode of '%s'. Must be either 'Fast' or 'Realtime'"
                                        .formatted(simMode));
                    }
                    queuedMessages.add(() -> {
                        robot.simulationSetMode(usersSimulationSpeed);
                    });
                });
        // Wait until startup has completed to ensure that the Webots simulator is
        // not still starting up.
        if (robot.step(0) == -1) {
            throw new RuntimeException("Couldn't even start up!");
        }
        SimRegisterer.connectDevices();


        // Pause the simulation until either the robot code tells us to proceed or the
        // user does.
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE);

        // Connect to the robot code on a separate thread. Does not block.
        try {
            wsConnection = WSConnection.connectHALSim(true);
        } catch (URISyntaxException e) {
            LOG.log(Level.ERROR,
                    "Error occurred connecting to server:" + e.getStackTrace());
            System.exit(1);
            return;
        }

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                wsConnection.object.closeBlocking();
            } catch (InterruptedException e) {
            }
        }));

        // Add a listener for robotTimeSec updates that runs the sim code to catch up and notifies
        // the robot of the new sim time.
        var robotTimeSecTopic = coordinator.getDoubleTopic("robotTimeSec");
        robotTimeSecTopic.setCached(false);
        DoubleSubscriber robotTimeSecSubscriber =
                robotTimeSecTopic.subscribe(-1.0, pubSubOptions);
        inst.addListener(robotTimeSecSubscriber,
                EnumSet.of(Kind.kValueAll, Kind.kImmediate),
                (event) -> {
                    final double robotTimeSec =
                            event.valueData.value.getDouble();
                    LOG.log(Level.DEBUG,
                            "Received robotTimeSec=%g".formatted(robotTimeSec));
                    queuedMessages.add(() -> {
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
                            boolean isDone = (robot.step(basicTimeStep) == -1);

                            simTimeSec = robot.getTime();
                            LOG.log(Level.DEBUG, "Sending simTimeSec of %g"
                                    .formatted(simTimeSec));
                            if (simTimeSecPublisher == null) {

                                LOG.log(Level.WARNING,
                                        "simTimeSecPublisher == null. We are probably disconnected from the robot.");
                            } else {
                                simTimeSecPublisher.set(simTimeSec);
                            }
                            inst.flush();
                            if (isDone) {
                                isDoneFuture.complete(true);
                                break;
                            }
                            Simulation.runPeriodicMethods();
                            // Regularly report the position, rotation, and/or velocity of the
                            // requested nodes
                            reportKinematicsForAllWatchedNodes(watchedNodes,
                                    robot);
                        }
                    });
                });

        // When a connection is (re-)established with the server, tell it that we are ready. When
        // the server sees that message, it knows that we will receive future messages.
        inst.addConnectionListener(true, (event) -> {
            LOG.log(Level.DEBUG, "In connection listener");
            if (event.is(Kind.kConnected)) {
                queuedMessages.add(() -> {
                    var reloadStatusTopic =
                            coordinator.getStringTopic("reloadStatus");
                    reloadStatusPublisher =
                            reloadStatusTopic.publish(pubSubOptions);
                    reloadStatusTopic.setCached(false);
                    LOG.log(Level.DEBUG, "Setting reloadStatus to Completed");
                    reloadStatusPublisher.set("Completed");
                    var simTimeSec = robot.getTime();

                    // Create a publisher for communicating the sim time and request that updates to
                    // it are
                    // communicated as quickly as possible.
                    var simTimeSecTopic =
                            coordinator.getDoubleTopic("simTimeSec");
                    simTimeSecPublisher =
                            simTimeSecTopic.publish(pubSubOptions);
                    LOG.log(Level.DEBUG, "Sending initial simTimeSec of %g"
                            .formatted(simTimeSec));
                    simTimeSecPublisher.set(simTimeSec);
                    simTimeSecTopic.setCached(false);
                    inst.flush();
                });
                LOG.log(Level.INFO,
                        "Connected to NetworkTables server '%s' at %s:%s"
                                .formatted(event.connInfo.remote_id,
                                        event.connInfo.remote_ip,
                                        event.connInfo.remote_port));
            } else if (event.is(Kind.kDisconnected)) {
                queuedMessages.add(() -> {
                    if (isWorldLoading)
                        return;
                    LOG.log(Level.INFO,
                            "Disconnected from NetworkTables server so closing publishers and pausing simulation.");
                    closePublishers();
                    waitUntilFlushed();
                    inst.stopClient();
                    inst.close();
                    inst = NetworkTableInstance.getDefault();
                    startNetworkTablesClient(inst);
                    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE);
                });
            }
        });

        startNetworkTablesClient(inst);

        // Process messages until simulation finishes
        try {
            while (isDoneFuture.getNow(false).booleanValue() == false) {
                try {
                    Simulation.runPeriodicMethods();
                    // Process any pending user interface events.
                    if (robot.step(0) == -1) {
                        break;
                    }
                    if (!queuedMessages.isEmpty()) {
                        LOG.log(Level.DEBUG, "Processing next queued message");
                        queuedMessages.takeFirst().run();
                    } else if (robotTimeSecSubscriber.exists()) {
                        // We are expecting a message from the robot that will tell us when to step
                        // the simulation.
                        LOG.log(Level.DEBUG,
                                "Waiting up to 1 second for a new message");
                        var msg = queuedMessages.pollFirst(1, TimeUnit.SECONDS);
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
                        LOG.log(Level.DEBUG,
                                "Simulation is not paused and no robot code in control. Calling robot.step(basicTimeStep)");

                        if (robot.step(basicTimeStep) == -1) {
                            break;
                        }
                        LOG.log(Level.DEBUG,
                                "robot.step(basicTimeStep) returned");
                    } else {
                        // The simulation is paused and robot code isn't in control so wait a beat
                        // before checking again (so we don't suck up all the CPU)
                        LOG.log(Level.DEBUG,
                                "Simulation is paused and no robot code in control. Sleeping for a step");
                        Thread.sleep(basicTimeStep);
                    }
                } catch (WebsocketNotConnectedException notConnectedException) {
                    try (var sw = new StringWriter();
                            var pw = new PrintWriter(sw)) {
                        notConnectedException.printStackTrace(pw);
                        LOG.log(Level.WARNING,
                                "No halsim connection to the robot code. Waiting 1 second in case it is restarting. Here is the stacktrace: %s"
                                        .formatted(sw.toString()));
                        Thread.sleep(1000);
                    }
                }
            }
        } catch (Throwable ex) {
            try (var sw = new StringWriter(); var pw = new PrintWriter(sw)) {
                ex.printStackTrace(pw);
                LOG.log(Level.ERROR,
                        "Exception while waiting for simulation to be done. Here is the stacktrace: %s"
                                .formatted(sw.toString()));
            }
            throw new RuntimeException(
                    "Exception while waiting for simulation to be done", ex);
        }

        LOG.log(Level.DEBUG, "Shutting down DeepBlueSim...");

        System.exit(0);
    }

    private static void closePublishers() {
        if (simTimeSecPublisher != null)
            simTimeSecPublisher.close();
        simTimeSecPublisher = null;
        if (reloadStatusPublisher != null)
            reloadStatusPublisher.close();
        reloadStatusPublisher = null;
        for (var entry : publisherByTopic.entrySet()) {
            entry.getValue().close();
        }
        publisherByTopic.clear();
    }

    private static void reportKinematicsForAllWatchedNodes(
            NetworkTable watchedNodes, final Supervisor robot) {
        // Note: we can't use watchedNodes.getSubTables() because it returns an empty array,
        // presumably because all of the topics within the subtables are uncached?
        for (var defPath : defPathsToPublish) {
            var node = robot.getFromDef(defPath);
            if (node == null) {
                LOG.log(Level.ERROR,
                        "Could not find node for the following DEF path: "
                                + defPath);
                continue;
            }
            var subTable = watchedNodes.getSubTable(defPath);
            if (subTable == null) {
                LOG.log(Level.WARNING,
                        "Could not find subtable for DEF path '%s' so not publishing it anymore "
                                .formatted(defPath));
                defPathsToPublish.remove(defPath);
                continue;
            }
            reportKinematicsFor(node, subTable);
        }
    }

    private static void reportKinematicsFor(Node node, NetworkTable subTable) {
        reportPositionFor(node, subTable);
        reportRotationFor(node, subTable);
        reportVelocityFor(node, subTable);

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
        var velocityTopic = subTable.getDoubleArrayTopic("velocity");
        var publisher = getPublisherByTopic(velocityTopic);
        publisher.set(node.getVelocity().clone());
        inst.flush();
    }

    private static void reportRotationFor(Node node, NetworkTable subTable) {
        var rotationTopic = subTable.getDoubleArrayTopic("rotation");
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
                    "rotation of %s has at least one non-finite elements. xyzArray = %s based on node.getOrientation() = %s"
                            .formatted(defPath, Arrays.toString(xyzArray),
                                    Arrays.toString(nodeOrientation)));
        }
        if (LOG.isLoggable(Level.DEBUG)) {
            LOG.log(Level.DEBUG,
                    "Setting rotation of %s to [%g, %g, %g]".formatted(
                            defPathForTopic(rotationTopic), xyzArray[0],
                            xyzArray[1], xyzArray[2]));
        }
        publisher.set(xyzArray);
        inst.flush();
    }

    private static void reportPositionFor(Node node, NetworkTable subTable) {
        var positionTopic = subTable.getDoubleArrayTopic("position");
        var publisher = getPublisherByTopic(positionTopic);
        double[] pos = node.getPosition().clone();
        if (LOG.isLoggable(Level.DEBUG)) {
            LOG.log(Level.DEBUG,
                    "Setting position of %s to [%g, %g, %g]".formatted(
                            defPathForTopic(positionTopic), pos[0], pos[1],
                            pos[2]));
        }
        publisher.set(pos);
        inst.flush();
    }

    private static String defPathForTopic(Topic topic) {
        var topicComponents = topic.getName().split("/");
        var defPath = topicComponents[topicComponents.length - 2];
        return defPath;
    }
}
