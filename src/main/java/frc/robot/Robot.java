/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.drivetrain;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  public static final XboxController driveController = new XboxController(RobotMap.PRIMARY_JOYSTICK);

  drivetrain m_drivetrain = drivetrain.getInstance();

  @Override
  public void robotInit() {
    m_drivetrain.robotInit();
  }

  @Override
  public void teleopPeriodic() {
    m_drivetrain.teleopPeriodic();
  }
}
