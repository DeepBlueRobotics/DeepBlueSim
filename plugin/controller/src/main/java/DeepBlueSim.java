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
public class DeepBlueSim {

    private static final ConcurrentLinkedDeque<Runnable> queuedMessages = new ConcurrentLinkedDeque<>();

    private static ScopedObject<Pair<String, StringCallback>> callbackStore = null;
    public static void main(String[] args) {
        ConnectionProcessor.setThreadExecutor(queuedMessages::add);
        final Supervisor robot = new Supervisor();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
        int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());
        
        SimConfig.initConfig();
        Simulation.init(robot, basicTimeStep);

        // Use a SimDeviceSim to coordinate with robot code tests
        final SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
        // Regular report the simulated robot's position
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

        // Wait until one timestep has completed to ensure that the Webots simulator is 
        // not still starting up.
        if (robot.step(basicTimeStep) == -1) {
            throw new RuntimeException("Couldn't even do one timestep!");
        }
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
