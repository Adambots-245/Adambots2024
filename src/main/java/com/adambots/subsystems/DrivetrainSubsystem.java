// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.VisionConstants;
import com.adambots.RobotMap;
import com.adambots.sensors.BaseGyro;
import com.adambots.utils.ModuleMap;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final BaseGyro m_gyro;

  //Pose Estimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Boolean frontLimelightFlag = false;
  private int inc = 0;

  // Odometry class for tracking robot pose
  private HashMap<ModulePosition, SwerveModule> swerveModules;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, BaseGyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;
    frontLimelightFlag = false;

    System.out.println(this.getName() + "Initializing");

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules), 
      new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.9, 0.9, 6));
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
    inc++;
    // Update the position of the robot on the ShuffleBoard field
    // Constants.odomField.setRobotPose(getPose());
    // System.out.println(this.getName() + ".periodic()");

    m_poseEstimator.updateWithTime(System.currentTimeMillis()/1000, m_gyro.getContinuousYawRotation2d(), ModuleMap.orderedModulePositions(swerveModules));

    // Update the odometry in the periodic block
    // if (inc % 10 == 0) {
    if (!DriverStation.isAutonomous()) {
      if (frontLimelightFlag && VisionHelpers.isDetected(VisionConstants.defaultAprilLimelite)) {
        Pose2d visionPose = VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite);
        if (visionPose.getY() > 1 && getContinuousAngleError(visionPose.getRotation().getRadians(), m_gyro.getContinuousYawRad()) < Math.toRadians(20) && VisionHelpers.getAprilHorizDist(VisionConstants.defaultAprilLimelite) < 4.5) {
          m_poseEstimator.addVisionMeasurement(visionPose, System.currentTimeMillis()/1000);
        }
      } 
      // else if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      //   Pose2d visionPose = VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite);
      //   if (visionPose.getY() > 1 && getContinuousAngleError(visionPose.getRotation().getRadians(), m_gyro.getContinuousYawRad()) < Math.toRadians(20) && VisionHelpers.getAprilHorizDist(VisionConstants.aprilLimelite) < 4.5) {
      //     m_poseEstimator.addVisionMeasurement(visionPose, System.currentTimeMillis()/1000);
      //   }
      // }
    }

    // Constants.frontLLField.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.defaultAprilLimelite));
    // Constants.rearLLField.setRobotPose(VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite));
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