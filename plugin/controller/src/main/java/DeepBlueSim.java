import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;
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

import org.ejml.simple.SimpleMatrix;
import org.java_websocket.client.WebSocketClient;
import org.team199.deepbluesim.SimRegisterer;
import org.team199.deepbluesim.Simulation;
import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.RunningObject;
import org.team199.wpiws.connection.WSConnection;

import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches
// the
// the name of the jar.
public class DeepBlueSim {

    private static final BlockingDeque<Runnable> queuedMessages =
            new LinkedBlockingDeque<>();

    private static RunningObject<WebSocketClient> wsConnection = null;

    /**
     * The time in milliseconds since the last timestep update from the robot.
     */
    private static volatile long lastStepMillis = 0;

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

    private static Map<String, DoubleArrayPublisher> positionPublisherByDefPath =
            new HashMap<>();
    private static Map<String, DoubleArrayPublisher> rotationPublisherByDefPath =
            new HashMap<>();
    private static Map<String, DoubleArrayPublisher> velocityPublisherByDefPath =
            new HashMap<>();

    private static final PrintStream log;

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static int ntLogLevel = LogMessage.kInfo;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

    static {
        try {
            File logFile = new File("DeepBlueSim.log");
            System.out.println("Logging to " + logFile.getAbsolutePath());
            log = new PrintStream(new FileOutputStream(logFile, true), true);
        } catch (FileNotFoundException ex) {
            throw new RuntimeException(ex);
        }
    }
    public static void main(String[] args) throws IOException {

        log.println("Starting log");
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
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        if (ntLogLevel > 0)
            inst.addLogger(ntLogLevel, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                log.println("NT instance log level %d message: %s(%d): %s"
                        .formatted(event.logMessage.level,
                                event.logMessage.filename,
                                event.logMessage.line,
                                event.logMessage.message));
            });
        inst.startClient4("Webots controller");
        inst.setServer("localhost");
        NetworkTable watchedNodes = inst.getTable("/DeepBlueSim/WatchedNodes");

        watchedNodes.addSubTableListener((parent, name, t) -> {
            defPathsToPublish.add(name);
        });

        ConnectionProcessor.setThreadExecutor(queuedMessages::add);

        final Supervisor robot = new Supervisor();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));

        if (!robot.getSupervisor()) {
            System.err.println(
                    "The robot does not have supervisor=true. This is required to detect devices.");
            System.exit(1);
        }
        // Get the basic timestep to use for calls to robot.step()
        final int basicTimeStep = (int) Math.round(robot.getBasicTimeStep());

        Simulation.init(robot, robot.getBasicTimeStep());

        updateUsersSimulationSpeed(robot);
        final CompletableFuture<Boolean> isDoneFuture =
                new CompletableFuture<Boolean>();

        // Regularly report the position, rotation, and/or velocity of the requested nodes
        Simulation.registerPeriodicMethod(() -> {
            for (var defPath : defPathsToPublish) {
                var node = robot.getFromDef(defPath);
                if (node == null) {
                    System.err.println(
                            "Could not find node for the following DEF path: "
                                    + defPath);
                    continue;
                }
                var subTable = watchedNodes.getSubTable(defPath);
                if (subTable == null) {
                    System.err.println(
                            "Could not find subtable for the following DEF path: "
                                    + defPath);
                    continue;
                }
                var positionTopic = subTable.getDoubleArrayTopic("position");
                if (positionTopic.exists()) {
                    var publisher = positionPublisherByDefPath.computeIfAbsent(
                            defPath, (key) -> positionTopic.publish());
                    publisher.set(node.getPosition());
                }
                var rotationTopic = subTable.getDoubleArrayTopic("rotation");
                if (rotationTopic.exists()) {
                    var simpleMatrix =
                            new SimpleMatrix(3, 3, true, node.getOrientation());
                    var rotation =
                            new Rotation3d(new Matrix<N3, N3>(simpleMatrix));
                    var publisher = rotationPublisherByDefPath.computeIfAbsent(
                            defPath, (key) -> rotationTopic.publish());
                    publisher.set(new double[] {rotation.getX(),
                            rotation.getY(), rotation.getZ()});
                }
                var velocityTopic = subTable.getDoubleArrayTopic("velocity");
                if (velocityTopic.exists()) {
                    var publisher = velocityPublisherByDefPath.computeIfAbsent(
                            defPath, (key) -> velocityTopic.publish());
                    publisher.set(node.getVelocity());
                }
            }
        });

        Timer simPauseTimer = new Timer();

        NetworkTable coordinator = inst.getTable("/DeepBlueSim/Coordinator");

        // Create a publisher for communicating the sim time and request that updates to it are
        // communicated as quickly as possible.
        DoublePublisher simTimeSecPublisher =
                coordinator.getDoubleTopic("simTimeSec").publish(
                        PubSubOption.sendAll(true),
                        PubSubOption.periodic(Double.MIN_VALUE));

        // Add a listener to handle reload requests.
        var reloadStatusTopic = coordinator.getStringTopic("reloadStatus");
        reloadStatusTopic.setCached(false);
        var reloadStatusEntry =
                reloadStatusTopic.getEntry("", PubSubOption.sendAll(true));
        inst.addListener(reloadStatusEntry.getTopic(),
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate),
                (event) -> {
                    final String reloadStatus =
                            event.valueData.value.getString();
                    log.println("In listener, reloadStatus = %s"
                            .formatted(reloadStatus));
                    if (!reloadStatus.equals("Requested"))
                        return;
                    queuedMessages.add(() -> {
                        // Cancel the sim pause timer so that it doesn't pause the
                        // simulation after we unpause it below.
                        simPauseTimer.cancel();
                        // Unpause before reloading so that the new controller can take it's
                        // first step.
                        robot.simulationSetMode(usersSimulationSpeed);

                        log.flush();
                        log.close();
                        robot.worldReload();
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
            System.err.println("Error occurred connecting to server:");
            e.printStackTrace(System.err);
            System.err.flush();
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
        DoubleSubscriber robotTimeSecSubscriber =
                coordinator.getDoubleTopic("robotTimeSec").subscribe(-1.0,
                        PubSubOption.sendAll(true),
                        PubSubOption.periodic(Double.MIN_VALUE));
        inst.addListener(robotTimeSecSubscriber.getTopic(),
                EnumSet.of(Kind.kValueAll, Kind.kImmediate),
                (event) -> {
                    final double robotTimeSec =
                            event.valueData.value.getDouble();
                    queuedMessages.add(() -> {
                        // Keep stepping the simulation forward until the sim time is more than
                        // the robot time or the simulation ends.
                        for (;;) {
                            double simTimeSec = robot.getTime();
                            if (simTimeSec > robotTimeSec) {
                                break;
                            }
                            // Unpause if necessary
                            robot.simulationSetMode(usersSimulationSpeed);
                            boolean isDone = (robot.step(basicTimeStep) == -1);

                            // If that was our first step, schedule a task to pause the
                            // simulator if it doesn't taken any steps for 1-2 seconds so it
                            // doesn't suck up CPU.
                            if (lastStepMillis == 0) {
                                simPauseTimer.schedule(new TimerTask() {
                                    @Override
                                    public void run() {
                                        if (System.currentTimeMillis()
                                                - lastStepMillis > 1000) {
                                            queuedMessages.add(() -> {
                                                updateUsersSimulationSpeed(
                                                        robot);
                                                robot.simulationSetMode(
                                                        Supervisor.SIMULATION_MODE_PAUSE);
                                            });
                                        }
                                    }
                                }, 1000, 1000);
                            }

                            lastStepMillis = System.currentTimeMillis();
                            simTimeSecPublisher.set(robot.getTime());
                            inst.flush();
                            if (isDone) {
                                isDoneFuture.complete(true);
                                break;
                            }
                            Simulation.runPeriodicMethods();
                        }
                    });
                });

        simTimeSecPublisher.set(robot.getTime());
        reloadStatusEntry.set("Completed");
        inst.flush();

        // Process messages until simulation finishes
        try {
            while (isDoneFuture.getNow(false).booleanValue() == false) {
                if (!queuedMessages.isEmpty()) {
                    // Either there is a message waiting or it is ok to wait for it because the
                    // robot code will tell us when to step the simulation.
                    queuedMessages.takeFirst().run();
                } else if (!robotTimeSecSubscriber.exists() && robot
                        .simulationGetMode() != Supervisor.SIMULATION_MODE_PAUSE) {
                    // The robot code isn't going to tell us when to step the simulation and the
                    // user has unpaused it.
                    Simulation.runPeriodicMethods();
                    if (robot.step(basicTimeStep) == -1) {
                        break;
                    }
                } else {
                    // The simulation is paused and robot code isn't in control so wait a beat
                    // before checking again (so we don't suck up all the CPU)
                    Thread.sleep(20);
                    // Process any pending user interface events.
                    if (robot.step(0) == -1) {
                        break;
                    }
                }
            }
        } catch (Exception ex) {
            throw new RuntimeException(
                    "Exception while waiting for simulation to be done", ex);
        }

        log.println("Shutting down DeepBlueSim...");

        System.exit(0);
    }
}
