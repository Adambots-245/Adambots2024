// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  TalonFX shooterWheel;
  

  double wheelSpeed = 0;

  public Shooter(TalonFX shooterWheel) {
    this.shooterWheel = shooterWheel;
  }

  public void setWheelSpeed(double newWheelSpeed){
    wheelSpeed = newWheelSpeed; 
  }


  @Override
  public void periodic() {
    shooterWheel.set(wheelSpeed);
  }
}
