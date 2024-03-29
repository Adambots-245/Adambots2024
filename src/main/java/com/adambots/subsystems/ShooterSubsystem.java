// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.RobotMap;
import com.adambots.Constants.ShooterConstants;
import com.adambots.actuators.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private BaseMotor shooterMotor;
  private BaseMotor shooterMotor2;

  private double shooterSpeed;  
  private double targetWheelSpeed;

  private PIDController pidController = new PIDController(0.0, 0.02, 0.0);

  public ShooterSubsystem(BaseMotor shooterMotor, BaseMotor shooterMotor2) {
    this.shooterMotor = shooterMotor;
    this.shooterMotor2 = shooterMotor2;

    shooterMotor.setInverted(true);
    shooterMotor.setBrakeMode(false);

    shooterMotor2.setInverted(false);
    shooterMotor2.setBrakeMode(false);

    shooterMotor2.setStrictFollower(RobotMap.shooterWheelPort);

    pidController.setIntegratorRange(0, 1);

    targetWheelSpeed = 0;

    Dash.add("Shooter Velocity", () -> getShooterVelocity());
    Dash.add("Target Shooter Velocity", () -> targetWheelSpeed);

    Dash.add("Shooter 1 Current", () -> shooterMotor.getCurrentDraw());
    Dash.add("Shooter 2 Current", () -> shooterMotor2.getCurrentDraw());
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
    return getShooterVelocity() > targetWheelSpeed - 1; // 1
  }

  @Override
  public void periodic() {
    if (targetWheelSpeed > 0) {
      shooterSpeed = pidController.calculate(getShooterVelocity(), targetWheelSpeed) + targetWheelSpeed/ShooterConstants.maxSpeed*1;
    } else {
      shooterSpeed = -0.07;
    }

    shooterMotor.set(shooterSpeed);
  }
}
