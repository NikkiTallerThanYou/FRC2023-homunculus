// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private static final Drivetrain m_drivetrain = new Drivetrain();

  public static final Arm m_arm = new Arm();

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_drivetrain::getPose,
    m_drivetrain::resetOdometry,
    m_drivetrain.m_ramseteController,
    m_drivetrain.m_kinematics,
    m_drivetrain::setSpeeds,
    AutonomousMap.eventMap,
    true,
    m_drivetrain
  );  
  public RobotContainer() {
    

    configureBindings();
    ntinst.startServer();

    
    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
      

    //   m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    //       m_driverController.getRightX() * Smodifier * Tmodifer);
    // }, m_drivetrain));


    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
    //   if (m_driverController.getLeftTriggerAxis()>0.5){
    //     m_drivetrain.curvatureDrive(m_driverController.getLeftY() * Smodifier, m_driverController.getRightX()*Smodifier, m_driverController.leftBumper().getAsBoolean());
    //   }else{
    //   m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    //       m_driverController.getRightX() * Smodifier);
    //   }
    // }, m_drivetrain));
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.42;
      double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5 && m_driverController.getLeftTriggerAxis()<0.5?1:0)*0.4); //
      

      m_drivetrain.arcadeDriveV(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())* Smodifier * Constants.maxSpeed,
          m_driverController.getRightX() * Math.abs(m_driverController.getRightX())* Smodifier * Constants.maxTurn);
    }, m_drivetrain));
   


  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("OneCargoAuto1", new PathConstraints(3, 4));
    return autoBuilder.fullAuto(pathGroup);
  }
}
