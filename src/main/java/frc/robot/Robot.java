// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Logger;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Drivetrain m_drivetrain;

  XboxController xboxController;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(5811);
    Logger.configureLoggingAndConfig(m_robotContainer, false); // true for verbose logging
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // double forwardSpeed;
    // double rotationSpeed;

    // forwardSpeed = -xboxController.getRightY();

    // if(xboxController.getAButton()){
    //   var result = camera.getLatestResult();
      
    //   if(result.hasTargets()){
    //     rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //     // double range = PhotonUtils.calculateDistanceToTargetMeters(rotationSpeed, rotationSpeed, forwardSpeed, rotationSpeed);
    //   } else { 
    //     rotationSpeed = 0; 
    //   }
    // } else {
    //   rotationSpeed = xboxController.getLeftX();
    // }

    // m_Drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
