// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  TalonFX shooterWheel;
  

  double wheelSpeed = 0;

  public ShooterSubsystem(TalonFX shooterWheel) {
    this.shooterWheel = shooterWheel;
    shooterWheel.setInverted(true);
  }

  public void setWheelSpeed(double newWheelSpeed){
    wheelSpeed = newWheelSpeed; 
  }


  @Override
  public void periodic() {
    shooterWheel.set(wheelSpeed);
  }
}
