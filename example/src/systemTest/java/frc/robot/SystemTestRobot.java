package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsManager;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.*;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
public class SystemTestRobot {

    @Test
    void testDrivesToLocation2() throws TimeoutException {
        testDrivesToLocation();
    }

    @Test
    void testDrivesToLocation() throws TimeoutException {
        try (var robot = new Robot(); var manager = new WebotsManager(robot)) {
            manager.withWorld("example/Webots/worlds/DBSExample.wbt")
                    .runAutonomous(Seconds.of(3.0))
                    .withNodePosition("ROBOT", (position) -> {
                        assertEquals(0.0,
                                position.getDistance(
                                        new Translation3d(-2.6, 0, 0)),
                                1.0, "Robot close to target position");
                    });
        }
    }
}
