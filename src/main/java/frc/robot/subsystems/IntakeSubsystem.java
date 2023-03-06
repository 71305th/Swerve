// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_LeftMotor = new CANSparkMax(IntakeConstants.kIntakeLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_RightMotor = new CANSparkMax(IntakeConstants.kIntakeRightMotorPort, MotorType.kBrushless);

  private final Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.kIntakeSolenoidChannel);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
