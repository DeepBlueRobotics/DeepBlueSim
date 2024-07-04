package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.io.FileNotFoundException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.math.util.Units;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
public class PWMMotorControllerTest {
    @Test
    void testShaftRotatesInAutonomous()
            throws TimeoutException, FileNotFoundException {
        // TODO: Fix the expected values to be physically correct and then fix PWMMotorMediator to
        // pass the test.
        try (var robot = new PWMMotorControllerRobot();
                var manager = new WebotsSimulator(
                        "../plugin/controller/src/webotsFolder/dist/worlds/PWMMotorController.wbt")) {
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
            }).run(robot);
        }
    }
}
