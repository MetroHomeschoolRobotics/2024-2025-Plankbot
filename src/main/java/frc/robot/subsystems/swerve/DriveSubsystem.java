package frc.robot.subsystems.swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.security.spec.MGF1ParameterSpec;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.swerve.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorID,
          DriveConstants.kFrontLeftTurningMotorID,
          DriveConstants.kFrontLeftTurningEncoderID,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorID,
          DriveConstants.kRearLeftTurningMotorID,
          DriveConstants.kRearLeftTurningEncoderID,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorID,
          DriveConstants.kFrontRightTurningMotorID,
          DriveConstants.kFrontRightTurningEncoderID,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorID,
          DriveConstants.kRearRightTurningMotorID,
          DriveConstants.kRearRightTurningEncoderID,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // SysID support
  // Mutable holders for logging values; persisted to avoid reallocation
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(), // default config is 1 volt/second ramp rate and 7 volt step voltage
      new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the motors.
        voltage -> {
          m_frontLeft.setDriveVoltage(voltage.magnitude());
          m_rearLeft.setDriveVoltage(voltage.magnitude());
          m_frontRight.setDriveVoltage(voltage.magnitude());
          m_rearRight.setDriveVoltage(voltage.magnitude());
        },
        log -> {
          // record a frame for each drive motor
          log.motor("drive-front-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontLeft.getDriveVoltage(), Volts))
            .linearPosition(
              m_distance.mut_replace(
                m_frontLeft.getDrivePosition(), Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontLeft.getDriveVelocity(), MetersPerSecond));
          
          log.motor("drive-rear-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearLeft.getDriveVoltage(), Volts))
            .linearPosition(
              m_distance.mut_replace(
                m_rearLeft.getDrivePosition(), Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearLeft.getDriveVelocity(), MetersPerSecond));
          
          log.motor("drive-front-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontRight.getDriveVoltage(), Volts))
            .linearPosition(
              m_distance.mut_replace(
                m_frontRight.getDrivePosition(), Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontRight.getDriveVelocity(), MetersPerSecond));
          
          log.motor("drive-rear-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearRight.getDriveVoltage(), Volts))
            .linearPosition(
              m_distance.mut_replace(
                m_rearRight.getDrivePosition(), Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearRight.getDriveVelocity(), MetersPerSecond));
          
        },
        this));


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("Drive Speed", getDriveVelocity());
    SmartDashboard.putNumber("Drive Voltage", getDriveVoltage());
    SmartDashboard.putNumber("Turn Voltage", m_rearLeft.getTurnVoltage());
    SmartDashboard.putNumber("Desired Rotation", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("Actual Heading", m_rearLeft.getAngleRadians());
    SmartDashboard.putNumber("Desired Speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("PID Error", m_rearLeft.getDrivePIDError());
    SmartDashboard.putNumber("PID Setpoint", m_rearLeft.getDrivePIDSetpoint());
    
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

public Command swerveDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, Boolean fieldRelative)
{
  return run(()->drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative)).withName("swerveDrive");
}

// Query current speed
public double getDriveVelocity()
{
  return m_rearLeft.getDriveVelocity();
}

// Query current voltage
public double getDriveVoltage()
{
  return m_rearLeft.getDriveVoltage();
}

  /* SysID Commands */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}