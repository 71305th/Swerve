// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Swerve extends SubsystemBase {

  private final WPI_Pigeon2 mImu = new WPI_Pigeon2(1);

  private final SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
  private SwerveDriveOdometry mOdometry;

  /** Creates a new Swerve. */
  public Swerve() {
    mLeftFrontModule = new SwerveModule(
      MotorConstants.kLeftFrontThrottleID, 
      MotorConstants.kLeftFrontRotorID, 
      MotorConstants.kLeftFrontRotorEncoderID, 
      MotorConstants.kLeftFrontRotorOffset,
      false
    );

    mRightFrontModule = new SwerveModule(
      MotorConstants.kRightFrontThrottleID, 
      MotorConstants.kRightFrontRotorID, 
      MotorConstants.kRightFrontRotorEncoderID, 
      MotorConstants.kRightFrontRotorOffset,
      false
    );

    mLeftRearModule = new SwerveModule(
      MotorConstants.kLeftRearThrottleID, 
      MotorConstants.kLeftRearRotorID, 
      MotorConstants.kLeftRearRotorEncoderID, 
      MotorConstants.kLeftRearRotorOffset,
      true
    );

    mRightRearModule = new SwerveModule(
      MotorConstants.kRightRearThrottleID, 
      MotorConstants.kRightRearRotorID, 
      MotorConstants.kRightRearRotorEncoderID, 
      MotorConstants.kRightRearRotorOffset,
      true
    );

    mOdometry = new SwerveDriveOdometry(
      MotorConstants.kSwerveKinematics,
      mImu.getRotation2d(),
      pos
    );
  }

  public static SwerveModulePosition[] pos = {
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
  };

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  // Updates odometry with current module state
    mOdometry.update(
      mImu.getRotation2d(), 
      new SwerveModulePosition[] {
        getModulePositions()[0], 
        getModulePositions()[1],
        getModulePositions()[2],
        getModulePositions()[3]
      }
    );
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      mLeftFrontModule.getState(), 
      mRightFrontModule.getState(), 
      mLeftRearModule.getState(), 
      mRightRearModule.getState()
    };
  }

  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      mLeftFrontModule.getPosition(), 
      mRightFrontModule.getPosition(), 
      mLeftRearModule.getPosition(), 
      mRightRearModule.getPosition()
    };
    }

  public void setModuleStates(SwerveModuleState[] desiredStates) {    
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
    mLeftFrontModule.setState(desiredStates[0]);
    mRightFrontModule.setState(desiredStates[1]);
    mLeftRearModule.setState(desiredStates[2]);
    mRightRearModule.setState(desiredStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
    SwerveModuleState[] states = null;
    if(fieldOriented) {
        states = MotorConstants.kSwerveKinematics.toSwerveModuleStates(
            // IMU used for field oriented control
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d())
        );
    } else {
        states = MotorConstants.kSwerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
        );
    }
    setModuleStates(states);
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }
  public void setPose(Pose2d pose, SwerveModulePosition[] modulepos ) {
    mOdometry.resetPosition(mImu.getRotation2d(), modulepos, pose);
  }
}
