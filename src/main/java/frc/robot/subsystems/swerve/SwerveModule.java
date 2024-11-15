// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.Constants.ModuleConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
  private final CANSparkFlex m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANcoder m_absoluteEncoder;

  // Drive Motors
  private final SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(ModuleConstants.kSModuleDriveController, 0);

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 
      ModuleConstants.kIModuleDriveController, 0);


  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorID The CAN ID of the drive motor.
   * @param turningMotorID The CAN ID of the turning motor.
   * @param absoluteEncoderID The CAN ID of the absolute encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int absoluteEncoderID,
      boolean turningEncoderReversed) {
    m_driveMotor = new CANSparkFlex(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, CANSparkLowLevel.MotorType.kBrushless);

    m_absoluteEncoder = new CANcoder(absoluteEncoderID);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotor.getEncoder().setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerRevolution); // Convert revolutions to m
    m_driveMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerRevolution / 60.0);  // Convert RPM to m/s

    m_driveMotor.setIdleMode(IdleMode.kBrake);




    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current Angle of the wheel
   * 
   * @return the current angle of the wheel
   */
  public double getAngleRadians() {
    return m_absoluteEncoder.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRadiansPerRevolution;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getEncoder().getVelocity(), new Rotation2d(getAngleRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getEncoder().getPosition(), new Rotation2d(getAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getAngleRadians());

    // Optimize the reference state to avoid spinning further than 90 degrees
    var optimizedState = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    optimizedState.speedMetersPerSecond *= optimizedState.angle.minus(new Rotation2d(getAngleRadians())).getCos();
    //desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(
          m_driveMotor.getEncoder().getVelocity(), 
          optimizedState.speedMetersPerSecond) 
        +
        m_driveFeedForward.calculate(optimizedState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
            getAngleRadians(), optimizedState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.setVoltage(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.getEncoder().setPosition(0.0);
    m_absoluteEncoder.setPosition(0.0);  // Do we really want to mess with the CANCoder?
  }

  // The following methods are for SysId only!!!
  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
  }

  // Returns units of Volts
  public double getDriveVoltage() {
    return m_driveMotor.get() * RobotController.getBatteryVoltage();
  }

  public double getDrivePIDSetpoint() {
    return m_driveMotor.get();
  }

  public double getDrivePIDError() {
    return m_drivePIDController.getPositionError();
  }

  // Returns units of meters
  public double getDrivePosition() {
    return m_driveMotor.getEncoder().getPosition();
  }

  // Returns units of meters/second
  public double getDriveVelocity() {
    return m_driveMotor.getEncoder().getVelocity();
  }
}