// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ArmConstants.State;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  BaseMotor shoulderMotor;
  BaseMotor wristMotor;
  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder wristEncoder;
  PIDController shoulderPID = new PIDController(0.0155, 0.000055, 0.0002);
  PIDController wristPID = new PIDController(0.013, 0, 0.00013);

  double shoulderLowerLimit = 120;
  double shoulderUpperLimit = 203;
  double wristLowerLimit = 160;
  double wristUpperLimit = 280;

  double shoulderAngle = 0;
  double wristAngle = 0;
  double shoulderSpeed, wristSpeed = 0;

  String currentState;

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    shoulderAngle = getShoulderAngle();
    wristAngle = getWristAngle();

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

  public void setCurrentState(String currentState) {
    this.currentState = currentState;
  }

  public String getCurrentState() {
    return currentState;
  }
  
  @Override
  public void periodic() {
    shoulderSpeed = shoulderPID.calculate(getShoulderAngle(), shoulderAngle);
    wristSpeed = wristPID.calculate(getWristAngle(), wristAngle);
    failSafes();
    shoulderMotor.set(shoulderSpeed);
    // shoulderMotor.set(0);
    wristMotor.set(Math.max(Math.min(wristSpeed, 0.3), -0.3));

        // System.out.println(getWristAngle());

  }

  private void failSafes() {
    // if(getShoulderAngle() < 151){
    //   wristLowerLimit = 276;
    // }else{
    //   wristLowerLimit = 160;
    // }
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
