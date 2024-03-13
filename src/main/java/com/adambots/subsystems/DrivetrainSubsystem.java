// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;

import com.adambots.Constants;
import com.adambots.Robot;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.RobotMap;
import com.adambots.sensors.BaseGyro;
import com.adambots.utils.ModuleMap;
import com.adambots.vision.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final BaseGyro m_gyro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  private HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules));

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig(
            new PIDConstants(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController),
            new PIDConstants(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController),
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            DriveConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                             // module.
            new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
        ), 
        () -> Robot.isOnRedAlliance(), //Flips path if on the red side of the field - ENSURE FIELD SIDE IS CORRECTLY SET IN DRIVERSTATION BEFORE TESTING AUTON
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getContinuousYawRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules)
    );

    // Update the position of the robot on the ShuffleBoard field
    Constants.field.setRobotPose(getPose());
    Constants.aprilTagfield.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue());
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
   * Resets the odometry and gyro to the specified pose, including x, y, and
   * heading.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    RobotMap.gyro.resetYawToAngle(pose.getRotation().getDegrees());
    m_odometry.resetPosition(pose.getRotation(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  public void resetOdometryXY(Pose2d pose) {
    m_odometry.resetPosition(new Rotation2d(RobotMap.gyro.getContinuousYawRad() + Math.PI), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *                      Speed (m/s) of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed (m/s) of the robot in the y direction (sideways).
   * @param rot
   *                      Angular rate of the robot.
   * @param fieldRelative
   *                      Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getContinuousYawRotation2d()));
    } else {
      setChassisSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
  }

  /**
   * Sets the swerve module states as according to the chassis speeds requested
   *
   * @param chassisSpeeds The desired ChassisSpeeds of the robot
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, desiredStates);
  }

  /**
   * Gets the chassis speeds of the robot as calculated from the swerve module
   * states
   *
   * @return The ChassisSpeeds of the robot
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(ModuleMap.orderedModuleStates(swerveModules));
  }

  /**
   * Stops the drivetrain
   */
  public void stop() {
    ModuleMap.stopModules(swerveModules);
  }
}