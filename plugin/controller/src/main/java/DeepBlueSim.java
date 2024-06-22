import java.io.IOException;
import java.lang.Thread.UncaughtExceptionHandler;
import java.net.URISyntaxException;

import org.java_websocket.client.WebSocketClient;
import org.team199.deepbluesim.SimRegisterer;
import org.team199.deepbluesim.Simulation;
import org.team199.deepbluesim.WebotsSupervisor;
import org.team199.wpiws.connection.RunningObject;
import org.team199.wpiws.connection.WSConnection;

import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class DeepBlueSim {

    private static RunningObject<WebSocketClient> wsConnection = null;

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

        final Supervisor robot = new Supervisor();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));

        if (!robot.getSupervisor()) {
            System.err.println("The robot does not have supervisor=true. This is required to detect devices.");
            System.exit(1);
        }

        // Get the basic timestep to use for calls to robot.step()
        final int basicTimeStep = (int) Math.round(robot.getBasicTimeStep());
        Simulation.init(robot, robot.getBasicTimeStep());

        // Connect to Network Tables
        {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            inst.startClient4("Webots controller");
            inst.setServer("localhost");
        }

        WebotsSupervisor.init(robot, basicTimeStep);

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
        {
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
        }

        try {
            // Process incoming messages until simulation finishes
            WebotsSupervisor.runUntilTermination(robot, basicTimeStep);
        } catch (Exception ex) {
            throw new RuntimeException("Exception while waiting for simulation to be done", ex);
        }

        System.out.println("Shutting down DeepBlueSim...");
        System.out.flush();

        System.exit(0);
    }

}
