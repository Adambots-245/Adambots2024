// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;
import java.util.Optional;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final BaseGyro m_gyro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getContinuousYawRad(), ModuleMap.orderedModulePositions(swerveModules));

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig(
            new PIDConstants(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController), 
            new PIDConstants(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController),
            DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            DriveConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
        ), 
        () -> { //Flips path if on the red side of the field - ENSURE FIELD SIDE IS CORRECTLY SET IN DRIVERSTATION BEFORE TESTING AUTON
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
        this // Reference to this subsystem to set requirements
    );

    // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetNoteOverride);
    // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetAprilOverride);
  }

  
  public Optional<Rotation2d> getRotationTargetNoteOverride(){
    if(VisionHelpers.isDetected(VisionConstants.noteLimelite)) {
      // Return an optional containing the rotation override (this should be a field relative rotation)
      if (Math.toRadians(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite)) > 0){
        return Optional.of(new Rotation2d(Math.toRadians(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite))));
      } else {
        return Optional.of(new Rotation2d(360 + Math.toRadians(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite))));
      }
    } else {
      // return an empty optional when we don't want to override the path's rotation
      return Optional.empty();
    }
  }

  public Optional<Rotation2d> getRotationTargetAprilOverride(){
    if(VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      // Return an optional containing the rotation override (this should be a field relative rotation)
      return Optional.of(new Rotation2d(Math.toRadians(VisionHelpers.getHorizAngle(VisionConstants.aprilLimelite))));
    } else {
      // return an empty optional when we don't want to override the path's rotation
      return Optional.empty();
    }
  }
   

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getContinuousYawRad(),
        ModuleMap.orderedModulePositions(swerveModules)
    );

    //Update the position of the robot on the ShuffleBoard field
    Constants.field.setRobotPose(getPose());
    Constants.aprilTagfield.setRobotPose(VisionHelpers.getAprilTagPose2d());
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
   * Resets the odometry and gyro to the specified pose, including x, y, and heading.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    RobotMap.gyro.resetYawToAngle(pose.getRotation().getDegrees());
    m_odometry.resetPosition(pose.getRotation(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed (m/s) of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed (m/s) of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the
   *          field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getContinuousYawRad()));
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
   * Gets the chassis speeds of the robot as calculated from the swerve module states
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