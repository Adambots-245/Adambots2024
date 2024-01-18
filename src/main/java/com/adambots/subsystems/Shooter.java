// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  TalonFX shooterWheel;
  TalonFX shooterAngleMotor;
  CANCoder shooterAngleEncoder;
  PIDController pid = new PIDController(0, 0, 0);


  double wheelSpeed = 0;
  double shooterAngle = 0;
  double shooterAngleSpeed = 0;

  public Shooter(TalonFX shooterWheel, TalonFX shooterAngleMotor, CANCoder shooterAngleEncoder) {
    this.shooterWheel = shooterWheel;
    this.shooterAngleMotor = shooterAngleMotor;
    this.shooterAngleEncoder = shooterAngleEncoder;
  }

  public void setWheelSpeed(double newWheelSpeed){
    wheelSpeed = newWheelSpeed; 
  }

  public void setShooterAngle(double newShooterAngle){
    shooterAngle = newShooterAngle;
  }


  @Override
  public void periodic() {
    shooterWheel.set(wheelSpeed);
    shooterAngleSpeed = pid.calculate(shooterAngleEncoder.getAbsolutePosition(), shooterAngle);
    shooterAngleMotor.set(shooterAngleSpeed);
  }
}
