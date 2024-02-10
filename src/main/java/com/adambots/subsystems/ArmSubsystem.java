// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  BaseMotor shoulderMotor;
  BaseMotor wristMotor;
  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder wristEncoder;
  PIDController shoulderPID = new PIDController(0.2, 0, 0);
  PIDController wristPID = new PIDController(0.04, 0, 0.01);

  double shoulderLowerLimit = 130;
  double shoulderUpperLimit = 200;
  double wristLowerLimit = 276;
  double wristUpperLimit = 280;

  double shoulderAngle = 0;
  double wristAngle = 0;
  double shoulderSpeed, wristSpeed = 0;

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    shoulderAngle = getShoulderAngle();
    wristAngle = 265;//getWristAngle();

    shoulderMotor.setInverted(false);
    wristMotor.setInverted(true);

    shoulderMotor.setNeutralMode(true);
    wristMotor.setNeutralMode(true);

    shoulderPID.enableContinuousInput(0, 360);
    wristPID.enableContinuousInput(0, 360);

    Dash.add("Shoulder Encoder", () -> getShoulderAngle());
    Dash.add("Wrist Encoder", () -> getWristAngle());
  }

  public void setShoulderAngle(double newShoulderAngle) {
    shoulderAngle = newShoulderAngle;
  }

  public void setWristAngle(double newWristAngle) {
    wristAngle = newWristAngle;
  }

  public void incrementShoulderAngle(double shoulderIncrement) {
    shoulderAngle += shoulderIncrement;

  }
    public void incrementWristAngle(double wristIncrement) {
    wristAngle += wristIncrement;
  }

  public double getWristAngle(){
    return wristEncoder.getAbsolutePosition() * 360;
  }

  public double getShoulderAngle(){
    return shoulderEncoder.getAbsolutePosition() * 360;
  }
  
  @Override
  public void periodic() {
    shoulderSpeed = shoulderPID.calculate(getShoulderAngle(), shoulderAngle);
    wristSpeed = wristPID.calculate(getWristAngle(), wristAngle);
    failSafes();
    //shoulderMotor.set(Math.max(Math.min(shoulderSpeed, 0.08), -0.08));
    shoulderMotor.set(0);
    wristMotor.set(Math.max(Math.min(wristSpeed, 0.1), -0.1));
  }

  private void failSafes() {
    if(getShoulderAngle() < 151){
      wristLowerLimit = 276;
    }else{
      wristLowerLimit = 160;
    }
    if (getShoulderAngle() > shoulderUpperLimit && shoulderSpeed > 0){
      shoulderAngle = shoulderUpperLimit;
    } 
    if(getShoulderAngle() < shoulderLowerLimit && shoulderSpeed < 0) {
        shoulderAngle = shoulderLowerLimit;
    }
    if (getWristAngle() < wristLowerLimit && wristSpeed < 0){
      wristAngle = wristLowerLimit;
    }
    if(getWristAngle() > wristUpperLimit && wristSpeed > 0){
        wristAngle = wristUpperLimit;
    }

  }
}
