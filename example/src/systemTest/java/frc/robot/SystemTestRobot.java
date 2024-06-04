package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.carlmontrobotics.libdeepbluesim.Watcher;
import org.carlmontrobotics.libdeepbluesim.WebotsManager;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
public class SystemTestRobot {

    @Test
    void testDrivesToLocationNewStyle2() throws TimeoutException {
        testDrivesToLocationNewStyle();
    }

    @Test
    void testDrivesToLocationNewStyle() throws TimeoutException {
        try (var robot = new Robot(); var manager = new WebotsManager(robot)) {
            manager.waitForUserToStart("example/Webots/worlds/DBSExample.wbt")
                    .runAutonomous(Seconds.of(3.0))
                    .withNodePosition("ROBOT", (position) -> {
                        assertEquals(0.0,
                                position.getDistance(
                                        new Translation3d(-2.6, 0, 0)),
                                1.0, "Robot close to target position");
                    });
        }
    }

    // @Test
    void testDrivesToLocation2() throws TimeoutException {
        testDrivesToLocation();
    }

    // @Test
    void testDrivesToLocation() throws TimeoutException {
        try (var testRobot = new Robot() {
            @Override
            public void startCompetition() {
                try (var webotsManager = new WebotsManager(this)) {
                    this.webotsManager = webotsManager;
                    // HAL must be initialized or SimDeviceSim.resetData() will crash and
                    // SmartDashboard
                    // might not work.
                    HAL.initialize(500, 0);
                    SimDeviceSim.resetData();
                    super.startCompetition();
                } finally {
                    SimDeviceSim.resetData();
                    // HAL.shutdown();
                }
            }

            Watcher robotWatcher;
            WebotsManager webotsManager;

            @Override
            public void simulationInit() {
                try {
                    webotsManager.waitForUserToStart(
                            "example/Webots/worlds/DBSExample.wbt");
                } catch (TimeoutException ex) {
                    throw new RuntimeException(ex);
                }
                robotWatcher = webotsManager.addWatcher("ROBOT");

                System.out.println("Webots has started. Enabling in autonomous.");
                // Simulate starting autonomous
                DriverStationSim.setAutonomous(true);
                DriverStationSim.setEnabled(true);
                DriverStationSim.notifyNewData();

                super.simulationInit();
            }

            @Override
            public void simulationPeriodic() {
                super.simulationPeriodic();

                if (webotsManager.getTimeSecs() > 3.0) {
                    // Simulate disabling the robot
                    DriverStationSim.setEnabled(false);
                    DriverStationSim.notifyNewData();

                    Translation3d actualPos = robotWatcher.getPosition();
                    System.out.println("robot position = " + actualPos);

                    Translation3d expectedPos = new Translation3d(-2.6, 0, 0);
                    var distance = expectedPos.getDistance(actualPos);

                    assertEquals(0.0, distance, 1.0, "Robot close to target position");

                    // Call endCompetition() to end the test and report success.
                    // NOTE: throwing an exception will end the test and report failure.
                    endCompetition();
                }
            }
        }) {
            testRobot.startCompetition();
        }
    }
}
