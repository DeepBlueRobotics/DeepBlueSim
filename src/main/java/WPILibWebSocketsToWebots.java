import java.net.URISyntaxException;
import java.util.concurrent.ConcurrentLinkedDeque;

import com.cyberbotics.webots.controller.Robot;

import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.WSConnection;
import org.team199.wpiws.devices.SimDeviceSim;

import code.SimConfig;
import code.lib.sim.Simulation;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class WPILibWebSocketsToWebots {

    private static final ConcurrentLinkedDeque<Runnable> queuedMessages = new ConcurrentLinkedDeque<>();
    
    public static void main(String[] args) {
        ConnectionProcessor.setThreadExecutor(queuedMessages::add);
        Robot robot = new Robot();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
        int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());
        
        SimConfig.initConfig();
        Simulation.init(robot, basicTimeStep);

        // Whenever a new connection is established to the robot code, tell it
        // that we're ready.
        ConnectionProcessor.addOpenListener(() -> {
            System.out.println("Creating a new SimDeviceSim(\"WebotsSupervisor\") ");
            SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
            webotsSupervisorSim.set("ready", "true");
        });
        try {
            WSConnection.connectHALSim(true);
        } catch(URISyntaxException e) {
            System.err.println("Error occured connecting to server:");
            e.printStackTrace(System.err);
            System.exit(1);
            return;
        }

        while(robot.step(basicTimeStep) != -1) {
            queuedMessages.forEach(Runnable::run);
            queuedMessages.clear();
            Simulation.runPeriodicMethods();
        }
    }
    
}
