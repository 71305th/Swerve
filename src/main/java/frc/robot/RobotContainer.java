// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.Intake.intakeGrab;
import frc.robot.commands.Intake.shootNode1;
import frc.robot.commands.Intake.shootNode2;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Initialize Joystick
  private final Joystick m_DriverJoystick = new Joystick(JoystickConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final Swerve m_Swerve = new Swerve();
  private final ManualDrive m_ManualDriveCommand = new ManualDrive(m_Swerve, m_DriverJoystick);
  private final Intake m_Intake = new Intake();
  private final ApriltagSubsystem m_ApriltagSubsystem = new ApriltagSubsystem();

  //comands
  private final shootNode1 m_Node1 = new shootNode1(m_Intake , m_ApriltagSubsystem, m_Swerve);
  private final shootNode2 m_Node2 = new shootNode2(m_Intake, m_ApriltagSubsystem, m_Swerve);
  private final intakeGrab m_IntakeGrab = new intakeGrab(m_Intake, m_DriverJoystick);

  public RobotContainer() {
    configureBindings();
    m_Swerve.setDefaultCommand(m_ManualDriveCommand);
  }

  private void configureBindings() {
    new JoystickButton(m_DriverJoystick, JoystickConstants.btn_A).onTrue(m_Node1);
    new JoystickButton(m_DriverJoystick, JoystickConstants.btn_B).onTrue(m_Node2);
    new JoystickButton(m_DriverJoystick, JoystickConstants.btn_X).onTrue(m_IntakeGrab);//must be btn x
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
