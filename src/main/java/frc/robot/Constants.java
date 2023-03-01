// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Voltage compensation
  public static final double kVoltageCompensation = 12.0;

  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int btn_A = 1;
    public static final int btn_B = 2;
    public static final int btn_X = 3;
    public static final int btn_Y = 4;
    public static final int btn_LB = 5;
    public static final int btn_RB = 6;
    public static final int btn_LS = 9;  
    public static final int btn_RS = 10;
  }

  public static class MotorConstants {

    // Rotor IDs
    public static final int kLeftFrontRotorID = 12;
    public static final int kRightFrontRotorID = 22;
    public static final int kLeftRearRotorID = 42;
    public static final int kRightRearRotorID = 32;

    // Throttle IDs
    public static final int kLeftFrontThrottleID = 11;
    public static final int kRightFrontThrottleID = 21;
    public static final int kLeftRearThrottleID = 41;
    public static final int kRightRearThrottleID = 31;
    
    // Rotor Encoder IDs
    public static final int kLeftFrontRotorEncoderID = 1;
    public static final int kRightFrontRotorEncoderID = 2;
    public static final int kLeftRearRotorEncoderID = 4;
    public static final int kRightRearRotorEncoderID = 3;

    // IMU ID
    public static final int kImuID = 0;
    
    // Rotor Offset
    public static final double kLeftFrontRotorOffset = 134.121;
    public static final double kRightFrontRotorOffset = -41.309;
    public static final double kLeftRearRotorOffset = 85.078;
    public static final double kRightRearRotorOffset = 48.076;

    // Rotor Inversion
    public static final boolean kRotorEncoderDirection = false;
    public static final boolean kRotorMotorInversion = false;

    // Swerve Kinematics (order: left front, right front, left rear, right rear)
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(0.288798, 0.288798), 
      new Translation2d(0.288798, -0.288798), 
      new Translation2d(-0.288798, 0.288798),
      new Translation2d(-0.288798, -0.288798)
    );
  }

  public static class PIDConstants {
    // Rotor PID constants
      public static final double kRotor_kP = 0.009;
      public static final double kRotor_kI = 0.0;
      public static final double kRotor_kD = 0;
  }

  public static class DriveConstants {
    // Max Speed / Acceleration
      public static final double kMaxVelocityMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecond = 6;
    
    // Wheel Diameter
      public static final double kWheelDiameterMeters = 0.1034;

    // Throttle Gear Ratio
      public static final double kThrottleGearRatio = 2.6470588;

    // Throttle Velocity Conversion Constant
    public static final double kSparkThrottleVelocityConversionFactor = 1 / kThrottleGearRatio / 60 * kWheelDiameterMeters * Math.PI; // Spark Max (NEO)
  }  
}
