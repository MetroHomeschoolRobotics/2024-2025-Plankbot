// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorID = 1;
    public static final int kRearLeftDriveMotorID = 7;
    public static final int kFrontRightDriveMotorID = 3;
    public static final int kRearRightDriveMotorID = 5;

    public static final int kFrontLeftTurningMotorID = 2;
    public static final int kRearLeftTurningMotorID = 8;
    public static final int kFrontRightTurningMotorID = 4;
    public static final int kRearRightTurningMotorID = 6;

    public static final int kFrontLeftTurningEncoderID = 1;
    public static final int kRearLeftTurningEncoderID = 4;
    public static final int kFrontRightTurningEncoderID = 2;
    public static final int kRearRightTurningEncoderID = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

  
    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 5.1;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);  // 4 inches
    public static final double kGearRatio = 6.12;  // SDS Mk4 L3
    public static final double kDriveEncoderDistancePerRevolution =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kGearRatio;

    public static final double kTurningEncoderRadiansPerRevolution =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI);

    public static final double kPModuleTurningController = 0.13; //0.5;
    public static final double kIModuleTurningController = 0.0;

    public static final double kPModuleDriveController = 0.4;
    public static final double kIModuleDriveController = 0.7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadBand = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}