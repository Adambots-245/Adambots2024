// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;


public class GyroFlipCommand extends Command {
  /** Creates a new GyroFlipCommand. */
  // Gyro gyro;

  public GyroFlipCommand() {
    // this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        System.out.println("Current Yaw At Reset: " + RobotMap.gyro.getContinuousYawDeg());
        RobotMap.gyro.setYawOffset(180);
        System.out.println("Yaw After At Reset: " + RobotMap.gyro.getContinuousYawDeg());
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}