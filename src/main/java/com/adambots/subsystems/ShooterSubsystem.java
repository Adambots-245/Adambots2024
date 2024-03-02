// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ShooterConstants;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  private BaseMotor shooterMotor;
  private double shooterSpeed;  
  private PIDController pid = new PIDController(0.0, 0.01, 0.0);
  
  boolean bool = false;
  private double targetWheelSpeed = 0;

  public ShooterSubsystem(BaseMotor shooterMotor) {
    this.shooterMotor = shooterMotor;

    shooterMotor.setInverted(true);
    shooterMotor.setNeutralMode(false);

    pid.setIntegratorRange(0, 1);

    Dash.add("Shooter Velocity", () -> getShooterVelocity());
    Dash.add("Shooter Command", () -> shooterSpeed);
    Dash.add("Shooter Target", () -> targetWheelSpeed);
  }

  public void setTargetWheelSpeed(double newWheelSpeed){
    targetWheelSpeed = newWheelSpeed; 
    shooterMotor.setNeutralMode(false);
    bool = false;
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
    /*
    if (targetWheelSpeed > 0 && getShooterVelocity() > 65) {
      shooterSpeed = 1;
    } else if (targetWheelSpeed > 0) {
      // shooterSpeed = pid.calculate(getShooterVelocity(), targetWheelSpeed) + targetWheelSpeed/ShooterConstants.maxSpeed*0.75;
      // shooterSpeed = targetWheelSpeed/ShooterConstants.maxSpeed*1;
      shooterSpeed = 0.9;
    } else if (targetWheelSpeed == 0 && getShooterVelocity() > 50) {
      shooterSpeed = 0.0;
    } else if (targetWheelSpeed == 0 && !bool) {
      bool = true;
      shooterMotor.setNeutralMode(true);
    }
    */

    if (targetWheelSpeed > 0) {
      shooterSpeed = pid.calculate(getShooterVelocity(), targetWheelSpeed) + targetWheelSpeed/ShooterConstants.maxSpeed*0.75;
      // shooterSpeed = targetWheelSpeed/ShooterConstants.maxSpeed*1;
    } else {
      shooterSpeed = -0.03;
    }

    // System.out.println(getShooterVelocity());

    // shooterSpeed = MathUtil.clamp(shooterSpeed, 0, 1);

    shooterMotor.set(shooterSpeed);
    // System.out.println(shooterSpeed);
  }
}
