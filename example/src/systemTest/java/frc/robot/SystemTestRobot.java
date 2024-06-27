package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.io.FileNotFoundException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
public class SystemTestRobot {

    @Test
    void testDrivesToLocationInAutonomous()
            throws TimeoutException, FileNotFoundException {
        try (var robot = new Robot();
                var manager =
                        new WebotsSimulator("Webots/worlds/DBSExample.wbt")) {
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
                        1.0, "Robot close to target position");
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.01, "Robot close to target velocity");
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
            }).run(robot);
        }
    }

    private volatile boolean stoppedTryingToTurn = false;

    @Test
    void testCanBeRotatedInPlaceInTeleop()
            throws TimeoutException, FileNotFoundException {
        try (var robot = new Robot();
                var manager =
                        new WebotsSimulator("Webots/worlds/DBSExample.wbt")) {
            manager.atSec(0.0, s -> {
                s.enableTeleop();
                DriverStationSim.setJoystickAxisCount(0, 2);
                DriverStationSim.setJoystickAxis(0, 1, 0.0);
                DriverStationSim.setJoystickAxis(0, 0, 0.15);
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
            }).atSec(1.0, s -> {
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.1, "Robot close to target velocity");
                assertEquals(19.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
                assertEquals(0.0,
                        new Translation3d(s.angularVelocity("ROBOT").getAxis())
                                .getDistance(new Translation3d(0, 0, 1)),
                        0.1, "Robot close to target angular velocity axis");
            }).atSec(4.0, s -> {
                assertEquals(45.0,
                        Units.radiansToDegrees(s.rotation("ROBOT").getZ()), 2.0,
                        "Robot close to target rotation");
                assertEquals(0.0,
                        s.velocity("ROBOT")
                                .getDistance(new Translation3d(0, 0, 0)),
                        0.01, "Robot close to target velocity");
                assertEquals(0.0,
                        Units.radiansToDegrees(
                                s.angularVelocity("ROBOT").getAngle()),
                        1, "Robot close to target angular velocity");
            }).run(robot);
        }
    }
}
