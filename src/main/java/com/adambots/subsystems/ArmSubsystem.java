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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  BaseMotor shoulderMotor;
  BaseMotor wristMotor;
  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder wristEncoder;
  PIDController shoulderPID = new PIDController(0.013, 0.000055, 0.00027);
  PIDController wristPID = new PIDController(0.013, 0, 0.00015);

  double shoulderLowerLimit = 115;
  double shoulderUpperLimit = 203;
  double wristLowerLimit = 160;
  double wristUpperLimit = 280;

  double targetShoulderAngle = ArmConstants.defaultState.getShoulderAngle();
  double targetWristAngle = ArmConstants.defaultState.getWristAngle();
  double shoulderSpeed, wristSpeed = 0;

  String currentStateName;

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    targetShoulderAngle = getShoulderAngle();
    targetWristAngle = getWristAngle();

    shoulderMotor.setInverted(false);
    wristMotor.setInverted(true);

    shoulderMotor.setNeutralMode(true);
    wristMotor.setNeutralMode(true);

    shoulderPID.enableContinuousInput(0, 360);
    wristPID.enableContinuousInput(0, 360);

    Dash.add("Shoulder Encoder", () -> getShoulderAngle());
    Dash.add("Wrist Encoder", () -> getWristAngle());
    Dash.add("wristSpeed",() ->  wristMotor.getVelocity());

  }

//   public void setTargetShoulderAngle(double newShoulderAngle) {
//     targetShoulderAngle = newShoulderAngle;
//   }

//   public void setTargetWristAngle(double newWristAngle) {
//     targetWristAngle = newWristAngle;
//   }

  public void incrementShoulderAngle(double shoulderIncrement) {
    targetShoulderAngle += shoulderIncrement;
  }

  public void incrementWristAngle(double wristIncrement) {
    targetWristAngle += wristIncrement;
  }

  public double getWristAngle(){
    return wristEncoder.getAbsolutePosition() * 360;
  }

  public double getShoulderAngle(){
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
    shoulderSpeed = shoulderPID.calculate(getShoulderAngle(), targetShoulderAngle);
    wristSpeed = wristPID.calculate(getWristAngle(), targetWristAngle);

    failSafes();

    wristSpeed = MathUtil.clamp(wristSpeed, -0.3, 0.3);

    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);
  }

  private void failSafes() {
    if(getShoulderAngle() < 160){
      wristLowerLimit = 271.5;
    }else{
      wristLowerLimit = 160;
    }
    if (getShoulderAngle() > shoulderUpperLimit && shoulderSpeed > 0){
      targetShoulderAngle = shoulderUpperLimit;
      shoulderSpeed = 0;
    } 
    if(getShoulderAngle() < shoulderLowerLimit && shoulderSpeed < 0) {
        targetShoulderAngle = shoulderLowerLimit;
        shoulderSpeed = 0;
    }
    if (getWristAngle() < wristLowerLimit && shoulderMotor.getVelocity() < 0){
      targetWristAngle = wristLowerLimit;
      wristSpeed = 0;
    }
    if(getWristAngle() > wristUpperLimit && shoulderMotor.getVelocity() > 0){
        targetWristAngle = wristUpperLimit;
        wristSpeed = 0;
    }
  }
}
