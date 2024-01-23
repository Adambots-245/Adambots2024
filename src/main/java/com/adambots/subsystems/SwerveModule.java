// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.ModuleConstants;
import com.adambots.sensors.AbsoluteEncoder;
import com.adambots.utils.Dash;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@SuppressWarnings("unused") //Gets rid of warning when m_position is unused, as m_position is still useful to have when debugging/testing
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, ModuleConstants.kDModuleDriveController);

  private final PIDController m_turningPIDController =
      new PIDController(ModuleConstants.kPModuleTurningController, 0, ModuleConstants.kDModuleTurningController);

  private ModulePosition m_position;

  /**
   * Constructs a SwerveModule.
   *
   * @param position The position of this module (front or back, right or left)
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param turningEncoderChannel The channels of the turning encoder.
   * @param driveMotorReversed Whether the drive motor is reversed.
   */
  public SwerveModule(ModulePosition position, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, boolean driveMotorReversed) {
    
    this.m_position = position; // Use position.name() to get the name of the position as a String

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setSmartCurrentLimit(32);
    m_driveMotor.enableVoltageCompensation(12.6);
    m_driveMotor.setInverted(driveMotorReversed);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityConversionFactor);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionConversionFactor);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setSmartCurrentLimit(21);
    m_turningMotor.enableVoltageCompensation(12.6);
    m_turningMotor.setInverted(true);

    m_turningEncoder = new AbsoluteEncoder(turningEncoderChannel);
    
    // Dash.add("Cancoder: " + m_position.name(), () -> m_absoluteEncoder.getAbsolutePositionDegrees());
    Dash.add("Wheel Speed: " + m_position.name(), () -> m_driveEncoder.getVelocity());

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double speedMetersPerSecond = m_driveEncoder.getVelocity();
    Rotation2d turnAngleRadians = m_turningEncoder.getAbsolutePositionRotation2D();

    return new SwerveModuleState(speedMetersPerSecond, turnAngleRadians);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition();
    Rotation2d turnAngleRadians = m_turningEncoder.getAbsolutePositionRotation2D();

    return new SwerveModulePosition(distance, turnAngleRadians);
  }

  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double speedMetersPerSecond = m_driveEncoder.getVelocity();
    double turnAngleRadians = m_turningEncoder.getAbsolutePositionRadians();

    // desiredState.speedMetersPerSecond *= desiredState.angle.minus(new Rotation2d(turnAngleRadians)).getCos(); //TODO: Test this

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnAngleRadians));

    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(speedMetersPerSecond, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.calculate(turnAngleRadians, state.angle.getRadians());

    //Set the motors to the calculated outputs
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes the SwerveModule drive encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }
}