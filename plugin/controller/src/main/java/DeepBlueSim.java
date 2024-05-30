import java.io.IOException;
import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;
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
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.ObjectCallback;

import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class DeepBlueSim {

    private static final BlockingDeque<Runnable> queuedMessages = new LinkedBlockingDeque<>();

    private static RunningObject<WebSocketClient> wsConnection = null;

    /**
     * Time value set by the simulator and the robot to indicate that the simulation should start.
     */
    private static final double START_SIMULATION = -2.0;

    /**
     * The time in milliseconds since the last timestep update from the robot.
     */
    private static volatile long lastStepMillis = 0;

    private static int usersSimulationSpeed = 0;

    // Remember the current simulation speed (default to real time if paused)
    private static void updateUsersSimulationSpeed(Supervisor robot) {
        usersSimulationSpeed = robot.simulationGetMode() == Supervisor.SIMULATION_MODE_PAUSE ? Supervisor.SIMULATION_MODE_REAL_TIME : robot.simulationGetMode();
    }

    private static Set<String> defPathsToPublish =
            new ConcurrentSkipListSet<String>();

    private static Map<String, DoubleArrayPublisher> positionPublisherByDefPath =
            new HashMap<>();
    private static Map<String, DoubleArrayPublisher> rotationPublisherByDefPath =
            new HashMap<>();
    private static Map<String, DoubleArrayPublisher> velocityPublisherByDefPath =
            new HashMap<>();

    public static void main(String[] args) throws IOException {
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

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("WebotsSupervisor");
        inst.startClient4("Webots controller");
        inst.setServer("localhost");

        table.addSubTableListener((parent, name, t) -> {
            defPathsToPublish.add(name);
        });

        ConnectionProcessor.setThreadExecutor(queuedMessages::add);

        final Supervisor robot = new Supervisor();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));

        if (!robot.getSupervisor()) {
            System.err.println("The robot does not have supervisor=true. This is required to detect devices.");
            System.exit(1);
        }
        // Get the basic timestep to use for calls to robot.step()
        final int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());

        Simulation.init(robot, robot.getBasicTimeStep());

        updateUsersSimulationSpeed(robot);
        // Use a SimDeviceSim to coordinate with robot code
        final CompletableFuture<Boolean> isDoneFuture =
                new CompletableFuture<Boolean>();
        final SimDeviceSim timeSynchronizerSim =
                new SimDeviceSim("TimeSynchronizer");

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
                var subTable = table.getSubTable(defPath);
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

        // Whenever the robot time changes, step the simulation until just past that time
        timeSynchronizerSim.registerValueChangedCallback("robotTimeSec", new ObjectCallback<String>() {
            @Override
            public synchronized void callback(String name, String value) {
                // Ignore null default initial value
                if (value == null)
                    return;

                double robotTimeSec = Double.parseDouble(value);

                // If we are asked to start the simulation, reload the world.
                // this will restart this controller process so that we are running the most recent controller.
                if (robotTimeSec == START_SIMULATION) {
                    // Cancel the sim pause timer so that it doesn't pause the simulation after we unpause it below.
                    simPauseTimer.cancel();
                    // Unpause before reloading so that the new controller can take it's first step.
                    robot.simulationSetMode(usersSimulationSpeed);
                    robot.worldReload();
                    return;
                }

                // Keep stepping the simulation forward until the sim time is more than the robot time
                // or the simulation ends.
                for(;;) {
                    double simTimeSec = robot.getTime();
                    if (simTimeSec > robotTimeSec) {
                        break;
                    }
                    // Unpause if necessary
                    robot.simulationSetMode(usersSimulationSpeed);
                    boolean isDone = (robot.step(basicTimeStep) == -1);

                    // If that was our first step, schedule a task to pause the simulator if it
                    // doesn't taken any steps for 1-2 seconds so it doesn't suck up CPU.
                    if (lastStepMillis == 0) {
                        simPauseTimer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                if (System.currentTimeMillis() - lastStepMillis > 1000) {
                                    queuedMessages.add(() -> {
                                        updateUsersSimulationSpeed(robot);
                                        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE);
                                    });
                                }
                            }
                        }, 1000, 1000);
                    }

                    lastStepMillis = System.currentTimeMillis();
                    timeSynchronizerSim.set("simTimeSec", robot.getTime());
                    if (isDone) {
                        isDoneFuture.complete(true);
                        break;
                    }
                    Simulation.runPeriodicMethods();
                }
            }
        }, true);


        // If the robot code starts before we us, then it might have already tried to tell
        // us it was ready and we would have missed it. So, we tell it we're ready when we
        // connect to it.
        ConnectionProcessor.addOpenListener(() -> {
            timeSynchronizerSim.set("simTimeSec", START_SIMULATION);
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

        // Connect to the robot code
        try {
            wsConnection = WSConnection.connectHALSim(true);
        } catch(URISyntaxException e) {
            System.err.println("Error occurred connecting to server:");
            e.printStackTrace(System.err);
            System.err.flush();
            System.exit(1);
            return;
        }

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                wsConnection.object.closeBlocking();
            } catch(InterruptedException e) {}
        }));

        // Process incoming messages until simulation finishes
        try {
            while (isDoneFuture.getNow(false).booleanValue() == false) {
                if (timeSynchronizerSim.get("robotTimeSec") != null || !queuedMessages.isEmpty()) {
                    // Either there is a message waiting or it is ok to wait for it because the
                    // robot code will tell us when to step the simulation.
                    queuedMessages.takeFirst().run();
                } else if (timeSynchronizerSim.get("robotTimeSec") == null
                        && robot.simulationGetMode() != Supervisor.SIMULATION_MODE_PAUSE) {
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
            throw new RuntimeException("Exception while waiting for simulation to be done", ex);
        }

        System.out.println("Shutting down DeepBlueSim...");
        System.out.flush();

        System.exit(0);
    }

}
