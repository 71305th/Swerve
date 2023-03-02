// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  // Initialize Joystick
  private final Joystick m_DriverJoystick = new Joystick(JoystickConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final Swerve m_Swerve = new Swerve();
  private final ManualDrive m_ManualDriveCommand = new ManualDrive(m_Swerve, m_DriverJoystick);
  private final ApriltagSubsystem m_apriltag = new ApriltagSubsystem();

  public RobotContainer() {
    configureBindings();
    m_Swerve.setDefaultCommand(m_ManualDriveCommand);
    m_apriltag.setDefaultCommand(new RunCommand(()->{
       Translation3d pos =  m_apriltag.getPosByApriltag();
       SmartDashboard.putNumber("X", pos.getX());
       SmartDashboard.putNumber("Y", pos.getY());
       SmartDashboard.putNumber("Z", pos.getZ());    
    }, m_apriltag));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
