package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.io.FileNotFoundException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.math.geometry.Translation3d;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
public class SystemTestRobot {

    @Test
    void testDrivesToLocation2()
            throws TimeoutException, FileNotFoundException {
        testDrivesToLocation();
    }

    @Test
    void testDrivesToLocation() throws TimeoutException, FileNotFoundException {
        try (var robot = new Robot();
                var manager =
                        new WebotsSimulator("Webots/worlds/DBSExample.wbt")) {
            manager.atSec(0.0, s -> {
                s.enableAutonomous();
            }).atSec(3.0, s -> {
                        assertEquals(0.0,
                        s.position("ROBOT").getDistance(
                                        new Translation3d(-2.6, 0, 0)),
                                1.0, "Robot close to target position");
            }).run(robot);
        }
    }
}
