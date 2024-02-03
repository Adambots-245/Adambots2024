// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.adambots.Tests.RobotStateMap;
import com.adambots.Tests.TestObjectFinder;
import com.adambots.sensors.RPLidarA1;
import com.adambots.utils.Units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlamSubsystem extends SubsystemBase {
  // private RPLidarA1 lidar;
  // private List<Translation2d> mPointcloud = new ArrayList<>();
  // private long mLastResetTime = 0;

  // /** Creates a new SlamSubsystem. */
  public SlamSubsystem() {
    TestObjectFinder tof = new TestObjectFinder();
    tof.interactivePointcloudTest();
  //    lidar = new RPLidarA1();
  //       lidar.setCallback((List<Translation2d> pointcloud) -> {
  //           synchronized (mPointcloud) {
  //               if (System.currentTimeMillis() - mLastResetTime > 0) {
  //                   mPointcloud = pointcloud;
  //                   mLastResetTime = System.currentTimeMillis();
  //               } else {
  //                   mPointcloud.addAll(pointcloud);
  //               }
  //           }
  //       }, new RobotStateMap(), new Transform2d(new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-14)), Rotation2d.fromDegrees(180)));
  //       lidar.start();

  //       Runtime.getRuntime().addShutdownHook(new Thread(() -> {
  //           lidar.stop();
  //           System.out.println("Graceful shutdown complete");
  //       }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // for (Translation2d translation2d : mPointcloud) {
    //   System.out.println(translation2d.getX() + " " + translation2d.getY());
    // }
  }
}
