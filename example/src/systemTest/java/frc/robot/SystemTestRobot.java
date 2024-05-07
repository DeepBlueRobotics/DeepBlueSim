package frc.robot;

import static org.junit.Assert.*;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.opencv.aruco.DetectorParameters;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

public class SystemTestRobot extends Robot {

    protected static final double MAX_TIME_DIFF_SECS = 0.0;

    public static void main(String... args) {
        RobotBase.startRobot(SystemTestRobot::new);
    }

    @Override
    public void startCompetition() {
        // Ensure that uncaught exceptions result in a non-zero exit status
        // so that failures can be detected by the caller (e.g. gradlew systemTest)
        try {
            super.startCompetition();
            HAL.shutdown();
            System.exit(0);
        } catch (Throwable throwable) {
            Throwable cause = throwable.getCause();
            if (cause != null) {
                throwable = cause;
            }
            DriverStation.reportError("Unhandled exception: " + throwable.toString(), throwable.getStackTrace());
            HAL.shutdown();
            System.exit(-1);
        }
    }

    SimDevice webotsSupervisor = null;
    SimDevice timeSynchronizer = null;
    SimDouble positionX = null, positionY = null, positionZ = null;
    SimDouble simTimeSecSim = null;
    SimDouble robotTimeSecSim = null;
    CompletableFuture<Boolean> isDoneFuture = new CompletableFuture<Boolean>();

    @Override
    public void simulationInit() {
        webotsSupervisor = SimDevice.create("WebotsSupervisor");
        timeSynchronizer = SimDevice.create("TimeSynchronizer");
        // SimDouble simStartMs = webotsSupervisor.createDouble("simStartMs", SimDevice.Direction.kInput, 0.0);
        // SimDouble robotStartMs = webotsSupervisor.createDouble("robotStartMs", SimDevice.Direction.kOutput, 0.0);
        simTimeSecSim = timeSynchronizer.createDouble("simTimeSec", SimDevice.Direction.kInput, -1.0);
        robotTimeSecSim = timeSynchronizer.createDouble("robotTimeSec", SimDevice.Direction.kOutput, -1.0);
        positionX = webotsSupervisor.createDouble("self.position.x", SimDevice.Direction.kInput, 0.0);
        positionY = webotsSupervisor.createDouble("self.position.y", SimDevice.Direction.kInput, 0.0);
        positionZ = webotsSupervisor.createDouble("self.position.z", SimDevice.Direction.kInput, 0.0);
        // SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");
        SimDeviceSim timeSynchronizerSim = new SimDeviceSim("TimeSynchronizer");

        // Wait for the Webots supervisor to be ready
        final var isReadyFuture = new CompletableFuture<Boolean>();
        // try (var callback = webotsSupervisorSim.registerValueChangedCallback(simStartMs, new SimValueCallback() {
        //     @Override
        //     public void callback(String name, int handle, int direction, HALValue value) {
        //         if (value.getDouble() > 0.0) {
        //             System.out.println("WebotsSupervisor is ready.");
        //             System.out.println("Telling WebotsSupervisor that we're ready");
        //             robotStartMs.set(System.currentTimeMillis());
        //             isReadyFuture.complete(true);
        //         }
        //     }
        // }, true))
        {
            // System.out.println("Telling WebotsSupervisor that we're ready");
            // robotStartMs.set(System.currentTimeMillis());
            // if (simStartMs.get() > 0.0) {
            //     System.out.println("WebotsSupervisor is ready");
            //     isReadyFuture.complete(true);
            // }

            final Notifier pauser = new Notifier(SimHooks::pauseTiming);

            timeSynchronizerSim.registerValueChangedCallback(simTimeSecSim, new SimValueCallback() {
                @Override
                public synchronized void callback(String name, int handle, int direction, HALValue value) {
                    double simTimeSec = value.getDouble();
                    System.out.println("simTime valueChangedCallback called with " + simTimeSec);
                    double robotTimeSec = Timer.getFPGATimestamp();
                    System.out.println("robotTimeSec = " + robotTimeSec);
                    double deltaSecs = simTimeSec + MAX_TIME_DIFF_SECS - robotTimeSec;
                    if (simTimeSec == -1.0) {
                        return;
                    }
                    if (robotTimeSecSim.get() == -2.0) {
                        // Waiting for sim to start
                        if (simTimeSec == -2.0) {
                            // Sim just started so restart robot timing

                            if (!isReadyFuture.isDone()) {
                                System.out.println("Calling isReadyFuture.complete(true)");
                                isReadyFuture.complete(true);
                            }
                        }
                    } else if (deltaSecs >= 0.0) {
                        // Tell the sim what the new robot time is. Strictly speaking it won't be that time
                        // until stepTiming() returns but this allows the sim to run in parrallel with the robot code.
                        // It's less deterministic but faster.
                        // System.out.println(String.format("calling robotTimeSec.set(%g)", Timer.getFPGATimestamp() + deltaSecs));
                        if (isDoneFuture.getNow(false).booleanValue()) {
                            // Call endCompetition() to end the test and report success.
                            // NOTE: throwing an exception will end the test and report failure.
                            System.out.println("Calling endCompetition()");
                            endCompetition();
                            System.out.println("endCompetition() returned");
                        } else {
                            robotTimeSec += deltaSecs;
                            // Let robot code run for deltaSecs. We use a Notifier instead of SimHooks.stepTiming() because 
                            // using SimHooks.stepTiming() causes accesses to sim data to block.
                            System.out.println("Calling pauser.stop()");
                            // Make any awaiting pauser run doesn't run before our new one.
                            pauser.stop();
                            System.out.println("Calling pauser.startSingle()");
                            pauser.startSingle(deltaSecs);
                            System.out.println("Calling resumeTiming()");
                            SimHooks.resumeTiming();
                            System.out.println("resumeTiming() returned");    
                            // System.out.println(String.format("Calling stepTiming(%g)", deltaSecs));
                            // SimHooks.stepTiming(deltaSecs);
                            // System.out.println("stepTiming() returned");    
                        }
                    }
                    if (robotTimeSec != robotTimeSecSim.get()) {
                        System.out.println("Calling robotTimeSecSim() with " + robotTimeSec);
                        robotTimeSecSim.set(robotTimeSec);
                    }
                }
            }, true);

            // Reset the clock. Without this, *Periodic calls that should have 
            // occurred while we waited, will be considered behind schedule and
            // will all happen at once.
            SimHooks.restartTiming();

            // Pause the clock so that we can step it in sync with the simulator
            SimHooks.pauseTiming();

            // Tell sim to start
            robotTimeSecSim.set(-2.0);
            
            // Tell sim that we're ready
            // robotTimeSecSim.set(Timer.getFPGATimestamp());

            // Wait up to 15 minutes for Webots to respond. On GitHub's MacOS Continuous
            // Integration servers, it can take over 8 minutes for Webots to start.
            var startedWaitingTimeMs = System.currentTimeMillis();
            var isReady = false;
            System.err.println("Waiting for WebotsSupervisor to be ready. Please open example/Webots/worlds/DBSExample.wbt in Webots.");
            while (!isReady && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
                try {
                    long elapsedTime = System.currentTimeMillis() - startedWaitingTimeMs;
                    long remainingTime = 900000 - elapsedTime;
                    System.out.println("Waiting for isReadyFuture");
                    if(remainingTime > 0) {
                        System.out.println("Waiting for isReadyFuture");
                        isReady = isReadyFuture.get(remainingTime, TimeUnit.MILLISECONDS);
                        System.out.println("isReadyFuture.get() returned " + isReady);
                    }
                    else isReady = true;
                } catch (TimeoutException ex) {
                    System.err.println("Waiting for WebotsSupervisor to be ready. Please open example/Webots/worlds/DBSExample.wbt in Webots.");
                } catch (InterruptedException|ExecutionException e) {
                    throw new RuntimeException("Error while waiting for WebotsSupervisor to be ready", e);
                }
            }
            System.out.println("Done waiting");
            assertTrue("Webots ready in time", isReady);
            System.out.println("Done asserting");
        }

        System.out.println("WebotsSupervisor is ready. Enabling in autonomous."); System.out.flush();

        // Simulate starting autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        super.simulationInit();
        System.out.println("Time at end of simulationInit="+Timer.getFPGATimestamp());
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Time at start of simulationPeriodic="+Timer.getFPGATimestamp());
        // if (Timer.getFPGATimestamp() > simTimeSecSim.get()) {
        //     SimHooks.pauseTiming();
        //     System.out.println(String.format("calling robotTimeSec.set(%g)", Timer.getFPGATimestamp()));
        //     robotTimeSec.set(Timer.getFPGATimestamp());
        // }

        super.simulationPeriodic();

        System.out.println("In simulationPeriodic()");
        System.out.println("Calling positionX.get()");
        System.out.println("self.position.x =" + positionX.get());
        // The motors are on for 2 secs. We wait an extra second to give the robot time to stop.
        if (Timer.getFPGATimestamp() > 3.0) {
            // Simulate disabling the robot
            System.out.println("Disabling robot");
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            System.out.println("Done disabling");

            Vector<N3> expectedPos = new Vector<>(N3.instance);
            expectedPos.set(0, 0, -2.6);
            expectedPos.set(1, 0, 0.0);
            expectedPos.set(2, 0, 0.0);

            System.out.println("Calling positionX.get()");
            System.out.println("self.position.x =" + positionX.get());
            System.out.println("self.position.y =" + positionY.get());
            System.out.println("self.position.z =" + positionZ.get());

            Vector<N3> actualPos = new Vector<>(N3.instance);
            actualPos.set(0, 0, positionX.get());
            actualPos.set(1, 0, positionY.get());
            actualPos.set(2, 0, positionZ.get());

            var diff = new Vector<N3>(expectedPos.minus(actualPos));
            var distance = Math.sqrt(diff.elementTimes(diff).elementSum());

            System.out.println("Asserting robot near targe position");
            assertEquals("Robot close to target position", 0.0, distance, 1.0);

            System.out.println("Calling isDoneFuture.complete(true)");
            isDoneFuture.complete(true);
        }
    }
}
