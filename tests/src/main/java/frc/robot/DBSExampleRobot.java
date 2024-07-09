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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class DBSExampleRobot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private DifferentialDrive m_robotDrive;
  private PWMMotorController m_leftMaster;
  private PWMMotorController m_leftFollower;
  private PWMMotorController m_rightMaster;
  private PWMMotorController m_rightFollower;
  private PWMMotorController m_elevator;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_leftMaster = new PWMVictorSPX(0);
    m_leftFollower = new PWMVictorSPX(1);
    m_leftMaster.addFollower(m_leftFollower);
    m_rightMaster = new PWMVictorSPX(2);
    m_rightFollower = new PWMVictorSPX(3);
    m_rightMaster.addFollower(m_rightFollower);

    m_rightMaster.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    m_elevator = new PWMVictorSPX(4);
  }

  public void close() {
    System.out.println("Closing motors in Robot.close()");
    m_leftMaster.close();
    m_leftFollower.close();
    m_rightMaster.close();
    m_rightFollower.close();
    m_elevator.close();
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
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
      m_elevator.set(0.3);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {}

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}
}
