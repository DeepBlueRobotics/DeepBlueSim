import java.net.URISyntaxException;
import java.util.concurrent.ConcurrentLinkedDeque;

import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

import org.team199.wpiws.Pair;
import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.WSConnection;
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.StringCallback;

import code.SimConfig;
import code.lib.sim.Simulation;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class WPILibWebSocketsToWebots {

    private static final ConcurrentLinkedDeque<Runnable> queuedMessages = new ConcurrentLinkedDeque<>();

    private static ScopedObject<Pair<String, StringCallback>> callbackStore = null;
    public static void main(String[] args) {
        System.out.println("Setting up thread executor"); System.out.flush();
        ConnectionProcessor.setThreadExecutor(queuedMessages::add);
        System.out.println("new Robot()"); System.out.flush();
        final Supervisor robot = new Supervisor();
        System.out.println("addShutdownHook"); System.out.flush();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
        int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());
        
        SimConfig.initConfig();
        Simulation.init(robot, basicTimeStep);

        System.out.println("Creating a new SimDeviceSim(\"WebotsSupervisor\") ");
        final SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");

        // When the robot code starts, we expect it to tell us it's ready, and we respond
        // that we're ready.
        callbackStore = webotsSupervisorSim.registerValueChangedCallback("robotStartMs", new StringCallback() {
				@Override
				public void callback(String name, String value) {
                    System.out.println("Telling the robot we're ready");
                    webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());                            
				}

        }, true);

        if (robot.getSupervisor()) {
            Simulation.registerPeriodicMethod(new Runnable() {
                public void run() {
                    Node self = robot.getSelf();
                    double[] pos = self.getPosition();
                    webotsSupervisorSim.set("self.position.x", pos[0]);
                    webotsSupervisorSim.set("self.position.y", pos[1]);
                    webotsSupervisorSim.set("self.position.z", pos[2]);
                }
            });
        } else {
            System.err.println("The robot does not have supervisor=true. Reporting is limited.");
        }

        // If the robot code started before we did, then it might have already tried to tell
        // us it was ready and we would have missed it. So, we tell it we're ready when we 
        // connect to it.
        ConnectionProcessor.addOpenListener(() -> {
            System.out.println("Telling the robot we're ready");
            webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());
        });
        try {
            System.out.println("Trying to connect to robot...");
            WSConnection.connectHALSim(true);
        } catch(URISyntaxException e) {
            System.err.println("Error occured connecting to server:");
            e.printStackTrace(System.err);
            System.err.flush();
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
