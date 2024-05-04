package frc.robot;

import static org.junit.Assert.*;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

public class SystemTestRobot extends Robot {

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
    SimDouble positionX = null, positionY = null, positionZ = null;
    SimDouble simTimeSec = null;

    @Override
    public void simulationInit() {
        webotsSupervisor = SimDevice.create("WebotsSupervisor");
        SimDouble simStartMs = webotsSupervisor.createDouble("simStartMs", SimDevice.Direction.kInput, 0.0);
        simTimeSec = webotsSupervisor.createDouble("simTimeSec", SimDevice.Direction.kInput, -1.0);
        positionX = webotsSupervisor.createDouble("self.position.x", SimDevice.Direction.kInput, 0.0);
        positionY = webotsSupervisor.createDouble("self.position.y", SimDevice.Direction.kInput, 0.0);
        positionZ = webotsSupervisor.createDouble("self.position.z", SimDevice.Direction.kInput, 0.0);
        SimDeviceSim webotsSupervisorSim = new SimDeviceSim("WebotsSupervisor");

        // Wait for the Webots supervisor to be ready
        final var future = new CompletableFuture<Boolean>();
        try (var callback = webotsSupervisorSim.registerValueChangedCallback(simStartMs, new SimValueCallback() {
            @Override
            public void callback(String name, int handle, int direction, HALValue value) {
                if (value.getDouble() > 0.0) {
                    System.out.println("WebotsSupervisor is ready");
                    future.complete(true);
                }
            }
        }, true)) {
            System.out.println("Telling WebotsSupervisor that we're ready");
            SimDouble robotStartMs = webotsSupervisor.createDouble("robotStartMs", SimDevice.Direction.kOutput, 0.0);
            robotStartMs.set(System.currentTimeMillis());
            if (simStartMs.get() > 0.0) {
                System.out.println("WebotsSupervisor is ready");
                future.complete(true);
            }
            // Wait up to 15 minutes for Webots to respond. On GitHub's MacOS Continuous
            // Integration servers, it can take over 8 minutes for Webots to start.
            var startedWaitingTimeMs = System.currentTimeMillis();
            var isReady = false;
            System.err.println("Waiting for WebotsSupervisor to be ready. Please open example/Webots/worlds/DBSExample.wbt in Webots.");
            while (!isReady && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
                try {
                    long elapsedTime = System.currentTimeMillis() - startedWaitingTimeMs;
                    long remainingTime = 900000 - elapsedTime;
                    if(remainingTime > 0) isReady = future.get(remainingTime, TimeUnit.MILLISECONDS);
                    else isReady = true;
                } catch (TimeoutException ex) {
                    System.err.println("Waiting for WebotsSupervisor to be ready. Please open example/Webots/worlds/DBSExample.wbt in Webots.");
                } catch (InterruptedException|ExecutionException e) {
                    throw new RuntimeException("Error while waiting for WebotsSupervisor to be ready", e);
                }
            }
            assertTrue("Webots ready in time", isReady);
        }

        // Reset the clock. Without this, *Periodic calls that should have 
        // occurred while we waited, will be considered behind schedule and
        // will all happen at once.
        SimHooks.restartTiming();

        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        webotsSupervisorSim.registerValueChangedCallback(simTimeSec, new SimValueCallback() {
            @Override
            public void callback(String name, int handle, int direction, HALValue value) {
                double deltaSecs = value.getDouble() - Timer.getFPGATimestamp();
                if (deltaSecs > 0.0) {
                    SimHooks.stepTimingAsync(deltaSecs);
                }
            }
        }, true);

        // Simulate starting autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        super.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        // The motors are on for 2 secs. We wait an extra second to give the robot time to stop.
        if (Timer.getFPGATimestamp() > 3.0) {
            // Simulate disabling the robot
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();

            Vector<N3> expectedPos = new Vector<>(N3.instance);
            expectedPos.set(0, 0, -2.6);
            expectedPos.set(1, 0, 0.0);
            expectedPos.set(2, 0, 0.0);

            System.out.println("self.position.x =" + positionX.get());
            System.out.println("self.position.y =" + positionY.get());
            System.out.println("self.position.z =" + positionZ.get());

            Vector<N3> actualPos = new Vector<>(N3.instance);
            actualPos.set(0, 0, positionX.get());
            actualPos.set(1, 0, positionY.get());
            actualPos.set(2, 0, positionZ.get());

            var diff = new Vector<N3>(expectedPos.minus(actualPos));
            var distance = Math.sqrt(diff.elementTimes(diff).elementSum());

            assertEquals("Robot close to target position", 0.0, distance, 1.0);

            // Call endCompetition() to end the test and report success.
            // NOTE: throwing an exception will end the test and report failure.
            endCompetition();
        }
    }
}
