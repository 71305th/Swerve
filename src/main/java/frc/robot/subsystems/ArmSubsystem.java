// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_ElbowLeft = new WPI_TalonFX(ArmConstants.kLeftElbowMotorPort);
  private final WPI_TalonFX m_ElbowRight = new WPI_TalonFX(ArmConstants.kRightElbowMotorPort);

  private final WPI_TalonFX m_Arm = new WPI_TalonFX(ArmConstants.kArmMotorPort);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_ElbowLeft.setSelectedSensorPosition(0);
    m_ElbowRight.setSelectedSensorPosition(0);
    m_Arm.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swingForward() {

  }

  public void swingBackward() {

  }

  public void stretch() {

  }

  public void retract() {
    
  }
}
