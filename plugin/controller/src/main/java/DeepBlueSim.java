import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.CompletableFuture;

import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

import org.team199.wpiws.Pair;
import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.RunningObject;
import org.team199.wpiws.connection.WSConnection;
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.StringCallback;
import org.java_websocket.client.WebSocketClient;
import org.team199.deepbluesim.SimRegisterer;
import org.team199.deepbluesim.Simulation;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class DeepBlueSim {

    private static final BlockingDeque<Runnable> queuedMessages = new LinkedBlockingDeque<>();

    @SuppressWarnings("unused")
    private static ScopedObject<Pair<String, StringCallback>> robotTimeSecCallbackStore = null;
    private static RunningObject<WebSocketClient> wsConnection = null;

    private static final double START_SIMULATION = -2.0;

    public static void main(String[] args) {
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

        // Use a SimDeviceSim to coordinate with robot code
        final CompletableFuture<Boolean> isDoneFuture = new CompletableFuture<Boolean>();
        final SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
        final SimDeviceSim timeSynchronizerSim = new SimDeviceSim("TimeSynchronizer");

        // Regularly report the simulated robot's position
        Simulation.registerPeriodicMethod(new Runnable() {
            public void run() {
                Node self = robot.getSelf();
                double[] pos = self.getPosition();
                webotsSupervisorSim.set("self.position.x", pos[0]);
                webotsSupervisorSim.set("self.position.y", pos[1]);
                webotsSupervisorSim.set("self.position.z", pos[2]);
            }
        });

        // Whenever the robot time changes, step the simulation until just past that time
        robotTimeSecCallbackStore = timeSynchronizerSim.registerValueChangedCallback("robotTimeSec", new StringCallback() {
            @Override
            public synchronized void callback(String name, String value) {
                // Ignore null default initial value
                if (value == null) 
                    return;

                double robotTimeSec = Double.parseDouble(value);

                // If we are asked to start the simulation, reload the world.
                // this will restart this controller process so that we are running the most recent controller.
                if (robotTimeSec == START_SIMULATION) {
                    robot.worldReload();
                    return;
                }

                // Keep stepping the simulation forward until the sim time is more than the robot time
                // or the simulation ends.
                while(true) {
                    double simTimeSec = robot.getTime();
                    if (simTimeSec > robotTimeSec) {
                        break;
                    }
                    boolean isDone = (robot.step(basicTimeStep) == -1);
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
                queuedMessages.takeFirst().run();
            }
        } catch (Exception ex) {
            throw new RuntimeException("Exception while waiting for simulation to be done");
        }

        System.out.println("Shutting down DeepBlueSim...");
        System.out.flush();

        System.exit(0);
    }

}
