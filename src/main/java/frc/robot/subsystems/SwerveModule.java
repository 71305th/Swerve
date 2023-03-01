// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ChenryLib.MathUtility;
import frc.robot.ChenryLib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDConstants;

public class SwerveModule extends SubsystemBase {
  // Initialize Rotor & Throttle Motor
  private CANSparkMax mRotor;
  private CANSparkMax mThrottle;

  // Initialize Encoder
  private WPI_CANCoder mRotorEncoder;

  // Initialize Rotor PID Controller
  private PID mRotorPID;

  /** Creates a new ExampleSubsystem. */
  /**
   * 
   * @param throttleID
   * @param throttleEncoderID
   * @param rotorID
   * @param rotorEncoderID
   * @param rotorOffsetAngelDeg
   */
  public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg ) {
    mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);
    mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
    
    mRotorEncoder = new WPI_CANCoder(rotorEncoderID);

    mThrottle.restoreFactoryDefaults();
    mRotor.restoreFactoryDefaults();
    mRotorEncoder.configFactoryDefault();

    mRotor.setInverted(MotorConstants.kRotorMotorInversion);
    mRotor.enableVoltageCompensation(Constants.kVoltageCompensation);
    mRotor.setIdleMode(IdleMode.kBrake);
    //mRotor.burnFlash();

    mRotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    mRotorEncoder.configMagnetOffset(rotorOffsetAngleDeg);
    mRotorEncoder.configSensorDirection(MotorConstants.kRotorEncoderDirection);
    mRotorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    mRotorPID = new PID(
      PIDConstants.kRotor_kP,
      PIDConstants.kRotor_kI,
      PIDConstants.kRotor_kD,
      30,0
    );

    //RotorPID = new PID(PIDConstants.kRotor_kP, PIDConstants.kRotor_kI, PIDConstants.kRotor_kD, 30, 0);

    mThrottle.enableVoltageCompensation(Constants.kVoltageCompensation);
    mThrottle.setIdleMode(IdleMode.kBrake);

  }

  /**
   * Return current state of module
   * 
   * @return module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        mThrottle.getEncoder().getVelocity() * DriveConstants.kSparkThrottleVelocityConversionFactor,
        Rotation2d.fromDegrees(mRotorEncoder.getAbsolutePosition())
    );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      mRotorEncoder.getPosition(),
      new Rotation2d(mThrottle.getEncoder().getPosition())
    );
  }

  /**
   * Set module state
   * 
   * @param state module state 
   */
  public void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);


    //normally its target - measurement but ok
    double error = getState().angle.getDegrees() - optimizedState.angle.getDegrees();
    double constrainedError = MathUtility.constrainAngleDegrees(error);
    double rotorOutput = mRotorPID.calculate(constrainedError);


    // System.out.println(state.angle.getDegrees());
    // System.out.println(optimizedState.angle.getDegrees());

    mRotor.set(rotorOutput);
    mThrottle.set(optimizedState.speedMetersPerSecond);
    // System.out.println(optimizedState.speedMetersPerSecond);
    // System.out.println(rotorOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
