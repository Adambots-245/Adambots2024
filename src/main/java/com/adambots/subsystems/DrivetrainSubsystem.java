// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.VisionConstants;
import com.adambots.Robot;
import com.adambots.RobotMap;
import com.adambots.sensors.BaseGyro;
import com.adambots.utils.ModuleMap;
import com.adambots.vision.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final BaseGyro m_gyro;

  //Pose Estimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Boolean frontLimelightFlag = false;

  // Odometry class for tracking robot pose
  private HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;
    frontLimelightFlag = false;

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
            new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
        ), 
        () -> Robot.isOnRedAlliance(), //Flips path if on the red side of the field - ENSURE FIELD SIDE IS CORRECTLY SET IN DRIVERSTATION BEFORE TESTING AUTON
        this // Reference to this subsystem to set requirements
    );

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules), 
      new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.9, 0.9, 6));
  }

  public double getContinuousAngleError (double setpoint, double measurement) {
    return MathUtil.inputModulus(setpoint - measurement, -Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      Pose2d visionPose = VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite);
      if (visionPose.getY() > 1 && getContinuousAngleError(visionPose.getRotation().getRadians(), RobotMap.gyro.getContinuousYawRad()) < Math.toRadians(20) && VisionHelpers.getAprilHorizDist(VisionConstants.aprilLimelite) < 4.5) {
        m_poseEstimator.addVisionMeasurement(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite), System.currentTimeMillis()/1000);
      }
    }
    if (VisionHelpers.isDetected(VisionConstants.defaultAprilLimelite) && frontLimelightFlag) {
      Pose2d visionPose = VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite);
      if (visionPose.getY() > 1 && getContinuousAngleError(visionPose.getRotation().getRadians(), RobotMap.gyro.getContinuousYawRad()) < Math.toRadians(20) && VisionHelpers.getAprilHorizDist(VisionConstants.defaultAprilLimelite) < 5) {
        m_poseEstimator.addVisionMeasurement(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite), System.currentTimeMillis()/1000);
        // System.out.println("DEFAULT ODOM UPDATE");
      }
    }
    m_poseEstimator.updateWithTime(System.currentTimeMillis()/1000, m_gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules));

    // Update the position of the robot on the ShuffleBoard field
    // Constants.field.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite));
    Constants.debugField.setRobotPose(getPose());
     
    Constants.aprilTagfield.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite));
  }

  public void setArmLimelightFlag (Boolean flagState) {
    frontLimelightFlag = flagState;
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry and gyro to the specified pose, including x, y, and
   * heading.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    RobotMap.gyro.resetYawToAngle(pose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(pose.getRotation(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  public void resetOdometryXY(Translation2d translation) {
    Pose2d pose = new Pose2d(translation.getX(), translation.getY(), new Rotation2d(RobotMap.gyro.getContinuousYawRad()));
    m_poseEstimator.resetPosition(pose.getRotation(), ModuleMap.orderedModulePositions(swerveModules), pose);
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
   * Stops the drivetrain
   */
  public void stop() {
    ModuleMap.stopModules(swerveModules);
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
}