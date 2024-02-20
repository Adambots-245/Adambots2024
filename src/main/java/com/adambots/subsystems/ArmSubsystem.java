// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  BaseMotor shoulderMotor;
  BaseMotor wristMotor;
  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder wristEncoder;
  PIDController shoulderPID = new PIDController(0.013, 0.000055, 0.00027);
  PIDController wristPID = new PIDController(0.013, 0, 0.00015);

  double shoulderLowerLimit = ArmConstants.shoulderLowerLimit;
  double shoulderUpperLimit = ArmConstants.shoulderUpperLimit;
  double wristLowerLimit = ArmConstants.wristLowerLimit;
  double wristUpperLimit = ArmConstants.wristUpperLimit;

  double targetShoulderAngle;
  double targetWristAngle;
  double shoulderSpeed, wristSpeed = 0;

  String currentStateName = "default";

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    targetShoulderAngle = getCurrentShoulderAngle();
    targetWristAngle = getCurrentWristAngle();

    shoulderMotor.setInverted(false);
    wristMotor.setInverted(false);

    shoulderMotor.setNeutralMode(true);
    wristMotor.setNeutralMode(true);

    shoulderPID.enableContinuousInput(0, 360);
    wristPID.enableContinuousInput(0, 360);

    Dash.add("Shoulder Encoder", () -> getCurrentShoulderAngle());
    Dash.add("Wrist Encoder", () -> getCurrentWristAngle());
    Dash.add("wristSpeed", () ->  wristMotor.getVelocity());
  }

  public void incrementShoulderAngle(double shoulderIncrement) {
    targetShoulderAngle += shoulderIncrement;
  }

  public void incrementWristAngle(double wristIncrement) {
    targetWristAngle += wristIncrement;
  }

  public double getCurrentWristAngle(){
    return wristEncoder.getAbsolutePosition() * 360;
  }

  public double getCurrentShoulderAngle(){
    return shoulderEncoder.getAbsolutePosition() * 360;
  }

  public void setCurrentState(State newState) {
    currentStateName = newState.getStateName();
    targetShoulderAngle = newState.getShoulderAngle();
    targetWristAngle = newState.getWristAngle();
  }

  public String getCurrentStateName() {
    return currentStateName;
  }
  
  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {shoulderSpeed = shoulderPID.calculate(getCurrentShoulderAngle(), targetShoulderAngle);}
    else {shoulderSpeed = 0;}
    if (DriverStation.isEnabled()) {wristSpeed = wristPID.calculate(getCurrentWristAngle(), targetWristAngle);}
    else {wristSpeed = 0;}

    failSafes();

    wristSpeed = MathUtil.clamp(wristSpeed, -0.3, 0.3);

    // shoulderMotor.set(shoulderSpeed);
    // wristMotor.set(wristSpeed);
    shoulderMotor.set(0);
    wristMotor.set(0);
  }

  private void failSafes() {
    if(getCurrentShoulderAngle() < ArmConstants.shoulderDangerZoneThreshold){
      wristLowerLimit = ArmConstants.wristDangerZoneLowerLimit;
    }else{
      wristLowerLimit = ArmConstants.wristLowerLimit;
    }

    if(getCurrentShoulderAngle() < ArmConstants.shoulderDangerZoneThreshold && getCurrentWristAngle() < wristLowerLimit && shoulderSpeed < 0){
      shoulderSpeed = 0;
    }

    if (getCurrentShoulderAngle() > shoulderUpperLimit && shoulderSpeed > 0){
      shoulderSpeed = 0;
    } 
    if(getCurrentShoulderAngle() < shoulderLowerLimit && shoulderSpeed < 0) {
      shoulderSpeed = 0;
    }

    if (getCurrentWristAngle() < wristLowerLimit && wristSpeed < 0){
      wristSpeed = 0;
    }
    if(getCurrentWristAngle() > wristUpperLimit && wristSpeed > 0){
      wristSpeed = 0;
    }
  }
}
