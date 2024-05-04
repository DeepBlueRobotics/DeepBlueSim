import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;
import java.util.concurrent.ConcurrentLinkedDeque;

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

    private static final ConcurrentLinkedDeque<Runnable> queuedMessages = new ConcurrentLinkedDeque<>();

    @SuppressWarnings("unused")
    private static ScopedObject<Pair<String, StringCallback>> callbackStore = null;
    private static RunningObject<WebSocketClient> wsConnection = null;

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
        Simulation.init(robot, robot.getBasicTimeStep());

        // Use a SimDeviceSim to coordinate with robot code
        final SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
        {
            // Regular report the simulated robot's position
            Simulation.registerPeriodicMethod(new Runnable() {
                public void run() {
                    Node self = robot.getSelf();
                    double[] pos = self.getPosition();
                    webotsSupervisorSim.set("self.position.x", pos[0]);
                    webotsSupervisorSim.set("self.position.y", pos[1]);
                    webotsSupervisorSim.set("self.position.z", pos[2]);
                }
            });


            // If the robot code starts after us, we expect it to tell us it's ready, and we respond
            // that we're ready.
            callbackStore = webotsSupervisorSim.registerValueChangedCallback("robotStartMs", new StringCallback() {
                @Override
                public void callback(String name, String value) {
                    System.out.println("Telling the robot we're ready");
                    webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());
                }
            }, true);

            // If the robot code starts before we us, then it might have already tried to tell
            // us it was ready and we would have missed it. So, we tell it we're ready when we
            // connect to it.
            ConnectionProcessor.addOpenListener(() -> {
                System.out.println("Telling the robot we're ready");
                webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());
            });
        }

        // Get the basic timestamp to use for calls to robot.step()
        int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());

        // Wait until one timestep has completed to ensure that the Webots simulator is
        // not still starting up.
        if (robot.step(basicTimeStep) == -1) {
            throw new RuntimeException("Couldn't even do one timestep!");
        }
        webotsSupervisorSim.set("simTimeSec", robot.getTime());

        SimRegisterer.connectDevices();

        // Connect to the robot code
        try {
            System.out.println("Trying to connect to robot...");
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

        while(robot.step(basicTimeStep) != -1) {
            webotsSupervisorSim.set("simTimeSec", robot.getTime());
            queuedMessages.forEach(Runnable::run);
            queuedMessages.clear();
            Simulation.runPeriodicMethods();
        }

        System.out.println("Shutting down DeepBlueSim...");
        System.out.flush();

        System.exit(0);
    }

}
