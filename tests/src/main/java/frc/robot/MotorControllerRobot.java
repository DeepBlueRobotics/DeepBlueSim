/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorConfig;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

public class MotorControllerRobot extends TimedRobot {

    private final Joystick m_stick = new Joystick(0);
    private final Timer m_timer = new Timer();
    private PWMMotorController m_pwmMotorController;
    private CANSparkMax m_sparkMaxMotorController;
    private CANSparkFlex m_sparkFlexMotorController;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_pwmMotorController = new PWMVictorSPX(1);
        m_sparkMaxMotorController =
                MotorControllerFactory.createSparkMax(2, MotorConfig.NEO);
        m_sparkFlexMotorController = MotorControllerFactory.createSparkFlex(3,
                MotorConfig.NEO_VORTEX);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public void close() {
        System.out.println("Closing motors in Robot.close()");
        m_pwmMotorController.close();
        m_sparkMaxMotorController.close();
        m_sparkFlexMotorController.close();
    }

    /**
     * This function is run once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // Run the motor at 50% for 2 seconds.
        if (m_timer.get() < 2.0) {
            m_pwmMotorController.set(0.5);
            m_sparkMaxMotorController.set(0.5);
            m_sparkFlexMotorController.set(0.5);
        } else {
            m_pwmMotorController.stopMotor();
            m_sparkMaxMotorController.stopMotor();
            m_sparkFlexMotorController.stopMotor();
        }
    }

    /**
     * This function is called periodically during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        m_pwmMotorController.set(m_stick.getX());
        m_sparkMaxMotorController.set(m_stick.getX());
        m_sparkFlexMotorController.set(m_stick.getX());
    }

}
