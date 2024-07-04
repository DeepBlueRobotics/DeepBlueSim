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
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class PWMMotorControllerRobot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private PWMMotorController m_motorController;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_motorController = new PWMVictorSPX(1);
  }

  public void close() {
    System.out.println("Closing motors in Robot.close()");
    m_motorController.close();
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
      m_motorController.set(0.5);
    } else {
      m_motorController.stopMotor();
    }
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_motorController.set(m_stick.getX());
  }

}