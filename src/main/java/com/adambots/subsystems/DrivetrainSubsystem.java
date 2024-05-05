// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.VisionConstants;
import com.adambots.Constants;
import com.adambots.RobotMap;
import com.adambots.sensors.BaseGyro;
import com.adambots.utils.ModuleMap;
import com.adambots.vision.LimelightHelpers;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final BaseGyro m_gyro;

  //Pose Estimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  // Odometry class for tracking robot pose
  private HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;

    System.out.println(this.getName() + "Initializing");

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules), 
      new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.9, 0.9, 6));

    m_poseEstimator.resetPosition(new Rotation2d(0), ModuleMap.orderedModulePositions(swerveModules), new Pose2d(0, 0, new Rotation2d(Math.PI)));
  }

  /**
   * Returns the shortest error between two angle measurements in radians, not matter whether angles are discrete or continuous.
   *
   * @param setpoint The first angle measurement in radians.
   * @param measurement The second angle measurement in radians.
   */
  public double getContinuousAngleError (double setpoint, double measurement) {
    return MathUtil.inputModulus(setpoint - measurement, -Math.PI, Math.PI);
  }

  @Override
  public void periodic() {

    // Update the position of the robot on the ShuffleBoard field
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules));

    // Update the odometry in the periodic block
    Pose2d visionPose = null;
    if (!DriverStation.isAutonomous()) {
      if (VisionHelpers.isDetected(VisionConstants.defaultAprilLimelite)) {
        // visionPose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.defaultAprilLimelite);
        visionPose = VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite);
        System.out.println("X: " + visionPose.getX() + " | Y: " + visionPose.getY() + " | Rot: " + visionPose.getRotation().getDegrees());
        // visionPose = new Pose2d(1.5 + (Math.random()-0.5)*20, 5 + (Math.random()-0.5)*20, new Rotation2d(Math.PI));
        // m_poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
      } 
    }

    Constants.odomField.setRobotPose(getPose());
    if (visionPose != null) {
      Constants.rearLLField.setRobotPose(visionPose);
    }
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
}