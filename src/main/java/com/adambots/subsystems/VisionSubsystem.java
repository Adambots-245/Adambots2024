// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.VisionConstants;
import com.adambots.utils.VisionHelpers;
import com.adambots.utils.VisionLookUpTable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

  }

  @Override
  public void periodic() {
    Constants.aprilTagfield.setRobotPose(VisionHelpers.getAprilTagPose2d());

    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      System.out.println(VisionLookUpTable.getShooterPreset(VisionHelpers.getAprilDistance()));
    }
  }
}
