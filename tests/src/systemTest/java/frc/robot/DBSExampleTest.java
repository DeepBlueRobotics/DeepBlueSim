package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.parallel.ResourceLock;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
@ResourceLock("WebotsSimulator")
public class DBSExampleTest {
    @Test
    // @RepeatedTest(value = 10, failureThreshold = 1) // Uncomment for stress testing.
    void testDrivesToLocationAndElevatesInAutonomous() throws Exception {
        try (var manager = new WebotsSimulator(
                "../plugin/controller/src/webotsFolder/dist/worlds/DBSExample.wbt",
                DBSExampleRobot::new)) {
            manager.atSec(0.0, s -> {
                s.enableAutonomous();
            }).atSec(1.0, s -> {
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(1.33, 0, 0)),
                        0.1, "Robot close to target velocity");
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
            }).atSec(3.0, s -> {
                assertEquals(0.0,
                        s.position("ROBOT")
                                .getDistance(new Translation3d(2.6, 0, 0)),
                        0.2, "Robot close to target position");
                assertEquals(0.0,
                        s.position("ELEVATOR")
                                .getDistance(new Translation3d(2.6, 0, 1.244)),
                                0.2, "Elevator close to target position");
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.05, "Robot close to target velocity");
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
            }).run();
        }
    }

    private volatile boolean stoppedTryingToTurn = false;

    @Test
    void testCanBeRotatedInPlaceInTeleop() throws Exception {
        try (var manager = new WebotsSimulator(
                "../plugin/controller/src/webotsFolder/dist/worlds/DBSExample.wbt",
                DBSExampleRobot::new)) {
            manager.atSec(0.0, s -> {
                s.enableTeleop();
                DriverStationSim.setJoystickAxisCount(0, 2);
                DriverStationSim.setJoystickAxis(0, 1, 0.0);
                DriverStationSim.setJoystickAxis(0, 0, 0.6);
                DriverStationSim.notifyNewData();
            }).everyStep(s -> {
                var yawDegrees =
                        Units.radiansToDegrees(s.rotation("ROBOT").getZ());
                if (yawDegrees > 45 && !stoppedTryingToTurn) {
                    DriverStationSim.setJoystickAxis(0, 1, 0.0);
                    DriverStationSim.setJoystickAxis(0, 0, 0.0);
                    DriverStationSim.notifyNewData();
                    stoppedTryingToTurn = true;
                }
            }).atSec(0.3, s -> {
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.1, "Robot close to target velocity");
                assertEquals(50.12, Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        5.0, "Robot close to target angular velocity");
                assertEquals(0.0,
                        new Translation3d(s.angularVelocity("ROBOT").getAxis())
                                .getDistance(new Translation3d(0, 0, 1)),
                        0.1, "Robot close to target angular velocity axis");
            }).atSec(1.5, s -> {
                // Note: Large tolerance because turning at a high speed means we can overshoot
                // significantly in 1 step.
                assertEquals(45.0,
                        Units.radiansToDegrees(s.rotation("ROBOT").getZ()),
                        10.0, "Robot close to target rotation");
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.1, "Robot close to target velocity");
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
            }).run();
        }
    }
}
