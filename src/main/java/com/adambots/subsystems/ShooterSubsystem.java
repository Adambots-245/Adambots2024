// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ShooterConstants;
import com.adambots.actuators.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private BaseMotor shooterMotor;

  private double shooterSpeed;  
  private double targetWheelSpeed;

  private PIDController pidController = new PIDController(0.0, 0.01, 0.0);

  public ShooterSubsystem(BaseMotor shooterMotor) {
    this.shooterMotor = shooterMotor;

    shooterMotor.setInverted(true);
    shooterMotor.setBrakeMode(false);

    pidController.setIntegratorRange(0, 1);

    targetWheelSpeed = 0;

    Dash.add("Shooter Velocity", () -> getShooterVelocity());
    Dash.add("Target Shooter Velocity", () -> targetWheelSpeed);
    // Dash.add("Shooter Command", () -> shooterSpeed);
    // Dash.add("Shooter Target", () -> targetWheelSpeed);
  }

  public void setTargetWheelSpeed(double newWheelSpeed){
    if (Math.abs(newWheelSpeed - targetWheelSpeed) > 10){
      pidController.reset();
    }
    targetWheelSpeed = newWheelSpeed; 
  }
  
  public double getShooterVelocity() {
    return shooterMotor.getVelocity();
  }

  public boolean isAtTargetSpeed() {
    if (DriverStation.isAutonomous()) {
      return Math.abs(getShooterVelocity() - targetWheelSpeed) < 3; // 1
    }
    return getShooterVelocity() > targetWheelSpeed - 3; // 1
  }

  @Override
  public void periodic() {
    if (targetWheelSpeed > 0) {
      shooterSpeed = pidController.calculate(getShooterVelocity(), targetWheelSpeed) + targetWheelSpeed/ShooterConstants.maxSpeed*0.5;
    } else {
      shooterSpeed = -0.03;
    }

    shooterMotor.set(shooterSpeed);
  }
}
