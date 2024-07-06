package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.parallel.ResourceLock;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.util.Units;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
@ResourceLock("WebotsSimulator")
public class PWMMotorControllerTest {
    @Test
    void testShaftRotatesInAutonomous() throws Exception {
        // TODO: Fix the expected values to be physically correct and then fix PWMMotorMediator to
        // pass the test.
        try (var manager = new WebotsSimulator(
                "../plugin/controller/src/webotsFolder/dist/worlds/PWMMotorController.wbt",
                PWMMotorControllerRobot.class)) {
            manager.atSec(0.0, s -> {
                s.enableAutonomous();
            }).atSec(1.0, s -> {
                assertEquals(175.5,
                        Units.radiansToDegrees(
                                s.angularVelocity("SHAFT").getAngle()),
                        1, "Shaft close to target angular velocity");
                assertEquals(160,
                        Units.radiansToDegrees(s.rotation("SHAFT").getZ()),
                        45.0, "Shaft close to target rotation");
            }).atSec(3.0, s -> {
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("SHAFT").getAngle()),
                        1, "Shaft close to target angular velocity");
                assertEquals(-28.8,
                        Units.radiansToDegrees(s.rotation("SHAFT").getZ()),
                        45.0, "Shaft close to target rotation");
            }).run();
        }
    }
}
