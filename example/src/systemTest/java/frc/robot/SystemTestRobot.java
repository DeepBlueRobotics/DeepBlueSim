package frc.robot;

import static org.junit.Assert.assertEquals;
import org.junit.Test;
import frc.robot.Robot;
import frc.robot.Main;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class SystemTestRobot extends Robot {

    public static void main(String... args) {
        RobotBase.startRobot(SystemTestRobot::new);
    }
    
    @Override
    public void simulationInit() {
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        super.simulationInit();
    }

    private int count = 0;

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        System.out.println(count);
        count++;
        if (count > 500) {
            DriverStationSim.setEnabled(false);
            endCompetition();
        }
    }
}
