import java.net.URISyntaxException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedDeque;

import com.cyberbotics.webots.controller.Robot;

import org.team199.wpiws.Pair;
import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.connection.ConnectionProcessor;
import org.team199.wpiws.connection.WSConnection;
import org.team199.wpiws.devices.SimDeviceSim;
import org.team199.wpiws.interfaces.SimDeviceCallback;
import org.team199.wpiws.interfaces.StringCallback;

import code.SimConfig;
import code.lib.sim.Simulation;

// NOTE: Webots expects the controller class to *not* be in a package and have a name that matches the
// the name of the jar.
public class WPILibWebSocketsToWebots {

    private static final ConcurrentLinkedDeque<Runnable> queuedMessages = new ConcurrentLinkedDeque<>();
    
    public static void main(String[] args) {
        System.out.println("Setting up thread executor"); System.out.flush();
        ConnectionProcessor.setThreadExecutor(queuedMessages::add);
        System.out.println("new Robot()"); System.out.flush();
        Robot robot = new Robot();
        System.out.println("addShutdownHook"); System.out.flush();
        Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
        int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());
        
        SimConfig.initConfig();
        Simulation.init(robot, basicTimeStep);

        // Whenever a new connection is established to the robot code, tell it
        // that we're ready.
        ConnectionProcessor.addOpenListener(() -> {
            System.out.println("Creating a new SimDeviceSim(\"WebotsSupervisor\") "); System.out.flush();
            SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");

            // Wait for robot to ask if we're ready
            final CompletableFuture<Boolean> future = new CompletableFuture<Boolean>();
            try (ScopedObject<Pair<String, StringCallback>> callback = webotsSupervisorSim.registerValueChangedCallback("areYouReady", new StringCallback() {
                @Override
                public void callback(String name, String value) {
                    System.out.println("areYouReady=" + value); System.out.flush();
                    if (Boolean.parseBoolean(value)) {
                        future.complete(true);
                    }
                }
            }, true)) {
                String areYouReady = webotsSupervisorSim.get("areYouReady");
                System.out.println("Initially, areYouReady=" + areYouReady); System.out.flush();
                if (Boolean.parseBoolean(areYouReady)) {
                    future.complete(true);
                } else {
                    System.out.println("Waiting for robot to ask if we're ready"); System.out.flush();
                }
                future.join();
            }

            System.out.println("Telling the robot we're ready"); System.out.flush();
            webotsSupervisorSim.set("ready", true);
        });
        try {
            System.out.println("Trying to connect to robot..."); System.out.flush();
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
