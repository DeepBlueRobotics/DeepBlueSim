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

    protected static final double MAX_TIME_DIFF_SECS = 0.0;

    @SuppressWarnings("unused")
    private static ScopedObject<Pair<String, StringCallback>> robotStartMsCallbackStore = null;
    private static ScopedObject<Pair<String, StringCallback>> robotTimeSecCallbackStore = null;
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
        // Get the basic timestep to use for calls to robot.step()
        final int basicTimeStep = (int)Math.round(robot.getBasicTimeStep());

        Simulation.init(robot, robot.getBasicTimeStep());

        // Use a SimDeviceSim to coordinate with robot code
        final CompletableFuture<Boolean> isDoneFuture = new CompletableFuture<Boolean>();
        final SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
        final SimDeviceSim timeSynchronizerSim = new SimDeviceSim("TimeSynchronizer");

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

            // Whenever the robot time changes, step the simulation until just past that time
            robotTimeSecCallbackStore = timeSynchronizerSim.registerValueChangedCallback("robotTimeSec", new StringCallback() {
                @Override
                public synchronized void callback(String name, String value) {
                    System.out.println("In robotTimeSec callback with value = " + value); System.out.flush();
            
                    if (value != null) {
                        double robotTimeSec = Double.parseDouble(value);

                        if (robotTimeSec == -2.0) {
                            System.out.println("Reloading world");
                            robot.worldReload();
                            return;
                        }
                        // Keep stepping the simulation forward until
                        // the sim time is more than MAX_TIME_DIFF_SECS ahead of the robot time
                        // or the simulation ends.
                        while(true) {
                            double simTimeSec = robot.getTime();
                            if (simTimeSec > robotTimeSec + MAX_TIME_DIFF_SECS) {
                                break;
                            }
                            System.out.println("Calling robot.step(basicTimeStep) in robotTimeSec callback"); System.out.flush();
                            boolean isDone = (robot.step(basicTimeStep) == -1);
                            System.out.println("robot.step(basicTimeStep) returned in robotTimeSec callback"); System.out.flush();
                            timeSynchronizerSim.set("simTimeSec", robot.getTime());
                            if (isDone) {
                                isDoneFuture.complete(true);
                                break;
                            }
                            Simulation.runPeriodicMethods();
                        }
                    }
                    System.out.println("exiting robotTimeSec callback"); System.out.flush();
                }
            }, true);


            // If the robot code starts after us, we expect it to tell us it's ready, and we respond
            // that we're ready.
            // robotStartMsCallbackStore = webotsSupervisorSim.registerValueChangedCallback("robotStartMs", new StringCallback() {
            //     @Override
            //     public void callback(String name, String value) {
            //         if (value != null) {
            //             System.out.println("Telling the robot we're ready");
            //             webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());
            //             isReadyFuture.complete(true);
            //         }
            //     }
            // }, true);

            // If the robot code starts before we us, then it might have already tried to tell
            // us it was ready and we would have missed it. So, we tell it we're ready when we
            // connect to it.
            ConnectionProcessor.addOpenListener(() -> {
                System.out.println("Telling the robot we're ready"); System.out.flush();
                // webotsSupervisorSim.set("simStartMs", System.currentTimeMillis());
                timeSynchronizerSim.set("simTimeSec", -2.0);
            });
        }
        // Wait until startup has completed to ensure that the Webots simulator is
        // not still starting up.
        System.out.println(String.format("Calling robot.step(%d)...", 0));
        System.out.flush();
        if (robot.step(0) == -1) {
            throw new RuntimeException("Couldn't even start up!");
        }
        System.out.println("Calling connectDevices()...");
        System.out.flush();
        SimRegisterer.connectDevices();


        // Connect to the robot code
        try {
            System.out.println("Trying to connect to robot...");
            System.out.flush();
            wsConnection = WSConnection.connectHALSim(true);
        } catch(URISyntaxException e) {
            System.err.println("Error occurred connecting to server:");
            e.printStackTrace(System.err);
            System.err.flush();
            System.exit(1);
            return;
        }

        System.out.println("connectHALSim() returned");

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                wsConnection.object.closeBlocking();
            } catch(InterruptedException e) {}
        }));

        // Tell robot we're ready
        System.out.println("telling robot we're ready. robot.getTime()=" + robot.getTime());
        timeSynchronizerSim.set("simTimeSec", -2.0);

        System.out.println("Waiting for simulation to finish...");
        System.out.flush();
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
