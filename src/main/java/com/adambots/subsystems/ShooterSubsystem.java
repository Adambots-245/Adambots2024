// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ShooterConstants;
import com.adambots.actuators.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  private BaseMotor shooterMotor;
  private double shooterSpeed;
  private PIDController pid = new PIDController(0.0, 0.01, 0.0);
  
  private double targetWheelSpeed = 0;

  public ShooterSubsystem(BaseMotor shooterWheel) {
    this.shooterMotor = shooterWheel;
    shooterWheel.setInverted(true);

    Dash.add("Shooter Velocity", () -> getShooterVelocity());
    Dash.add("Shooter Command", () -> shooterSpeed);
    Dash.add("Shooter Target", () -> targetWheelSpeed);
  }

  public void setTargetWheelSpeed(double newWheelSpeed){
    targetWheelSpeed = newWheelSpeed; 
    pid.reset();
  }
  

  public double getShooterVelocity() {
    return shooterMotor.getVelocity();
  }

  public boolean isAtTargetSpeed() {
    return targetWheelSpeed - getShooterVelocity() < 1;
  }

  @Override
  public void periodic() {
    if (targetWheelSpeed > 0) {
      shooterSpeed = pid.calculate(getShooterVelocity(), targetWheelSpeed) + targetWheelSpeed/ShooterConstants.maxSpeed*0.75;
    } else {
      shooterSpeed = 0;
    }

    shooterSpeed = MathUtil.clamp(shooterSpeed, 0, 1);

    shooterMotor.set(shooterSpeed);
  }
}
