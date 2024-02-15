// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  BaseMotor shooterWheel;
  

  double wheelSpeed = 0;
  public ShooterSubsystem(BaseMotor shooterWheel) {
    this.shooterWheel = shooterWheel;
    shooterWheel.setInverted(true);

    Dash.add("Shooter Velocity", () -> getShooterVelocity());
  }

  public void setWheelSpeed(double newWheelSpeed){
    wheelSpeed = newWheelSpeed; 
  }

  public double getShooterVelocity() {
    return shooterWheel.getVelocity();
  }


  @Override
  public void periodic() {
    shooterWheel.set(wheelSpeed);
  }
}
