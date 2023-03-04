// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final double kLeftFrontRotorOffset = 44.209;
    public static final double kRightFrontRotorOffset = 176.484;
    public static final double kLeftRearRotorOffset = -47.9;
    public static final double kRightRearRotorOffset = 137.549;

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
      public static final double kRotor_kP = 0.011;
      public static final double kRotor_kI = 0.0;
      public static final double kRotor_kD = 0.0025;
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
    /**
   * length: meters</p>
   * cone1-->lower one</p>
   * cone2--> upper one</p>
   */
  public static class ImageConstants{

    public static final double kXDis = 1;
    public static final double kYDis = 1;
    public static final double kZDis = 1;
    public static final double kApriltagHeight = 0.515;
    public static final double ConesHight_2 = 1.06600625;
    public static final double ConesHight_1 = 0.568325;
    public static final double DistenceBetweenCones = 0.4318;
    public static final Transform3d CameraToRobotCenter = new Transform3d(new Translation3d(), new Rotation3d(0,0,0));
  }

  
  public static final class FieldConstants {
    /**
     * length: meters </p>
     * </p>
     * ID position:</p>
     * |---blue--|----red----|</p>
     * |----5----------4-----|</p>
     * |----6----------3-----|</p>
     * |----7----------2-----|</p>
     * |----8----------1-----|</p>
     * @param targetID
     * @param position
     */
    public static Map<Integer, Translation3d> ApriltagMap = new HashMap<>(){{
        put(1, new Translation3d(15.69085, 1.07088214, 0.46355));
        put(2, new Translation3d(15.69085, 2.73911785, 0.46355));
        put(3, new Translation3d(15.69085, 4.40735356, 0.46355));
        put(4, new Translation3d(16.18615, 6.7262375, 0.695325));
        put(5, new Translation3d(0.3556, 6.7262375, 0.695325));
        put(6, new Translation3d(0.8509, 4.40735356, 0.46355));
        put(7, new Translation3d(0.8509, 2.73911785, 0.46355));
        put(8, new Translation3d(0.8509, 1.07088214, 0.46355));
    }};

    /**
     * length: meters </p>
     * </p>
     * Objects position:</p>
     * |---blue--|----red----|</p>
     * |----5----------4-----|</p>
     * |----6----------3-----|</p>
     * |------9----10--------|</p>
     * |----7----------2-----|</p>
     * |----8----------1-----|</p>
     * </p>
     * 9,10: charge station center
     * @param ObjectNumber
     * @param position
     */
    public static Map<Integer, Translation3d> ObjectMap = new HashMap<>(){{
      put(1, new Translation3d(9.4234, 0.92075, 0));
      put(2, new Translation3d(9.4234, 2.13995, 0));
      put(3, new Translation3d(9.4234, 3.35915, 0));
      put(4, new Translation3d(9.4234, 4.57835, 0));
      put(5, new Translation3d(7.96565, 4.57835, 0));
      put(6, new Translation3d(7.96565, 3.35915, 0));
      put(7, new Translation3d(7.96565, 2.13995, 0));
      put(8, new Translation3d(7.96565, 0.92075, 0));
      put(9, new Translation3d(2.3955375, 2.73911785, 0));
      put(10, new Translation3d(14.1462125, 0.92075, 0));
    }};

    /**
     * |----------blue---------------|----------------red-----------|</p>
     * | 609(19) 608(29) 607(39) ---------- 307(49) 308(59) 309(69) |</p>
     * | 606(18) 605(28) 604(38) ---------- 304(48) 305(58) 306(68) |</p>
     * | 603(17) 602(27) 601(37) ---------- 301(47) 302(57) 303(67) |</p>
     * | 709(16) 708(26) 707(36) ---------- 207(46) 208(56) 209(66) |</p>
     * | 706(15) 705(25) 704(35) ---------- 204(45) 205(55) 206(65) |</p>
     * | 703(14) 702(24) 701(34) ---------- 201(44) 202(54) 203(64) |</p>
     * | 809(13) 808(23) 807(33) ---------- 107(43) 108(53) 109(63) |</p>
     * | 806(12) 805(22) 804(32) ---------- 104(42) 105(52) 106(62) |</p>
     * | 803(11) 802(21) 801(31) ---------- 101(41) 102(51) 103(61) |</p>
     * @Unit meter
     * @param NodeNumber
     * @param position
     */
    public static Map<Integer, Translation3d> NodeMap = new HashMap<>(){{
      put(101, new Translation3d(15.3750, 0, 0));
      put(102, new Translation3d(15.7462, 0, 0.8636));
      put(103, new Translation3d(16.1735, 0, 1.169));
      put(104, new Translation3d(15.3750, 0, 0));
      put(105, new Translation3d(15.7462, 0, 0.5969));
      put(106, new Translation3d(16.1735, 0, 0.9017));
      put(107, new Translation3d(15.3750, 0, 0));
      put(108, new Translation3d(15.7462, 0, 0.8636));
      put(109, new Translation3d(16.1735, 0, 0.1169));
      put(201, new Translation3d(15.3750, 0, 0));
      put(202, new Translation3d(15.7462, 0, 0.8636));
      put(203, new Translation3d(16.1735, 0, 0.1169));
      put(204, new Translation3d(15.3750, 0, 0));
      put(205, new Translation3d(15.7462, 0, 0.5969));
      put(206, new Translation3d(16.1735, 0, 0.9017));
      put(207, new Translation3d(15.3750, 0, 0));
      put(208, new Translation3d(15.7462, 0, 0.8636));
      put(209, new Translation3d(16.1735, 0, 0.1169));
      put(301, new Translation3d(15.3750, 0, 0));
      put(302, new Translation3d(15.7462, 0, 0.8636));
      put(303, new Translation3d(16.1735, 0, 0.1169));
      put(304, new Translation3d(15.3750, 0, 0));
      put(305, new Translation3d(15.7462, 0, 0.8636));
      put(306, new Translation3d(16.1735, 0, 0.1169));
      put(307, new Translation3d(15.3750, 0, 0));
      put(308, new Translation3d(15.7462, 0, 0.5969));
      put(309, new Translation3d(16.1735, 0, 0.9017));
      put(601, new Translation3d(1.1747, 0, 0));
      put(602, new Translation3d(0.8001, 0, 0.8636));
      put(603, new Translation3d(0.3683, 0, 0.1169));
      put(604, new Translation3d(1.1747, 0, 0));
      put(605, new Translation3d(0.8001, 0, 0.8636));
      put(606, new Translation3d(0.3683, 0, 0.1169));
      put(607, new Translation3d(1.1747, 0, 0));
      put(608, new Translation3d(0.8001, 0, 0.5969));
      put(609, new Translation3d(0.3683, 0, 0.9017));
      put(701, new Translation3d(1.1747, 0, 0));
      put(702, new Translation3d(0.8001, 0, 0.8636));
      put(703, new Translation3d(0.3683, 0, 0.1169));
      put(704, new Translation3d(1.1747, 0, 0));
      put(705, new Translation3d(0.8001, 0, 0.5969));
      put(706, new Translation3d(0.3683, 0, 0.9017));
      put(707, new Translation3d(1.1747, 0, 0));
      put(708, new Translation3d(0.8001, 0, 0.8636));
      put(709, new Translation3d(0.3683, 0, 0.1169));
      put(801, new Translation3d(1.1747, 0, 0));
      put(802, new Translation3d(0.8001, 0, 0.8636));
      put(803, new Translation3d(0.3683, 0, 0.1169));
      put(804, new Translation3d(1.1747, 0, 0));
      put(805, new Translation3d(0.8001, 0, 0.5969));
      put(806, new Translation3d(0.3683, 0, 0.9017));
      put(807, new Translation3d(1.1747, 0, 0));
      put(808, new Translation3d(0.8001, 0, 0.8636));
      put(809, new Translation3d(0.3683, 0, 0.1169));

      put(41, new Translation3d(15.3750, 0, 0));
      put(51, new Translation3d(15.7462, 0, 0.8636));
      put(61, new Translation3d(16.1735, 0, 0.1169));
      put(42, new Translation3d(15.3750, 0, 0));
      put(52, new Translation3d(15.7462, 0, 0.5969));
      put(62, new Translation3d(16.1735, 0, 0.9017));
      put(43, new Translation3d(15.3750, 0, 0));
      put(53, new Translation3d(15.7462, 0, 0.8636));
      put(63, new Translation3d(16.1735, 0, 0.1169));
      put(44, new Translation3d(15.3750, 0, 0));
      put(54, new Translation3d(15.7462, 0, 0.8636));
      put(64, new Translation3d(16.1735, 0, 0.1169));
      put(45, new Translation3d(15.3750, 0, 0));
      put(55, new Translation3d(15.7462, 0, 0.5969));
      put(65, new Translation3d(16.1735, 0, 0.9017));
      put(46, new Translation3d(15.3750, 0, 0));
      put(56, new Translation3d(15.7462, 0, 0.8636));
      put(66, new Translation3d(16.1735, 0, 0.1169));
      put(47, new Translation3d(15.3750, 0, 0));
      put(57, new Translation3d(15.7462, 0, 0.8636));
      put(67, new Translation3d(16.1735, 0, 0.1169));
      put(48, new Translation3d(15.3750, 0, 0));
      put(58, new Translation3d(15.7462, 0, 0.8636));
      put(68, new Translation3d(16.1735, 0, 0.1169));
      put(49, new Translation3d(15.3750, 0, 0));
      put(59, new Translation3d(15.7462, 0, 0.5969));
      put(69, new Translation3d(16.1735, 0, 0.9017));
      put(37, new Translation3d(1.1747, 0, 0));
      put(27, new Translation3d(0.8001, 0, 0.8636));
      put(17, new Translation3d(0.3683, 0, 0.1169));
      put(38, new Translation3d(1.1747, 0, 0));
      put(28, new Translation3d(0.8001, 0, 0.8636));
      put(18, new Translation3d(0.3683, 0, 0.1169));
      put(39, new Translation3d(1.1747, 0, 0));
      put(29, new Translation3d(0.8001, 0, 0.5969));
      put(19, new Translation3d(0.3683, 0, 0.9017));
      put(34, new Translation3d(1.1747, 0, 0));
      put(24, new Translation3d(0.8001, 0, 0.8636));
      put(14, new Translation3d(0.3683, 0, 0.1169));
      put(35, new Translation3d(1.1747, 0, 0));
      put(25, new Translation3d(0.8001, 0, 0.5969));
      put(15, new Translation3d(0.3683, 0, 0.9017));
      put(36, new Translation3d(1.1747, 0, 0));
      put(26, new Translation3d(0.8001, 0, 0.8636));
      put(16, new Translation3d(0.3683, 0, 0.1169));
      put(31, new Translation3d(1.1747, 0, 0));
      put(21, new Translation3d(0.8001, 0, 0.8636));
      put(11, new Translation3d(0.3683, 0, 0.1169));
      put(32, new Translation3d(1.1747, 0, 0));
      put(22, new Translation3d(0.8001, 0, 0.5969));
      put(12, new Translation3d(0.3683, 0, 0.9017));
      put(33, new Translation3d(1.1747, 0, 0));
      put(23, new Translation3d(0.8001, 0, 0.8636));
      put(13, new Translation3d(0.3683, 0, 0.1169));
    }};
  }

  public static class pipelineConstants{
    
    //pipeline index
    public static final int Limelight1_apriltag = 0;
    public static final int Limelight1_limelightUpper = 1;
    public static final int Limelight1_limelightLower = 2;
  }
}

