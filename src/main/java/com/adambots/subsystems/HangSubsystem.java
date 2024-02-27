// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.HangConstants;
import com.adambots.actuators.BaseMotor;
import com.adambots.actuators.BaseSolenoid;
import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  
  private BaseMotor leftHangMotor;
  private BaseMotor rightHangMotor;
  private double leftHangMotorSpeed, rightHangMotorSpeed = 0;
  private BaseSolenoid leftBaseSolenoid;
  private BaseSolenoid rightBaseSolenoid;

  public HangSubsystem(BaseMotor leftHangMotor, BaseMotor rightHangMotor, BaseSolenoid rightBaseSolenoid, BaseSolenoid leftBaseSolenoid) {
    this.leftHangMotor = leftHangMotor;
    this.rightHangMotor = rightHangMotor;
    this.leftBaseSolenoid = leftBaseSolenoid;
    this.rightBaseSolenoid = rightBaseSolenoid;

    leftHangMotor.setInverted(true);
    rightHangMotor.setInverted(false);

    leftHangMotor.setNeutralMode(true);
    rightHangMotor.setNeutralMode(true);

    resetEncoders();

    Dash.add("Left Hang Pos", () -> getLeftMotorPosition());
    Dash.add("Right Hang Pos", () -> getRightMotorPosition());
  }

  public void setLeftMotorSpeed(double newLeftHangMotorSpeed){
    leftHangMotorSpeed = newLeftHangMotorSpeed;
  }

  public void setRightMotorSpeed(double newRightHangMotorSpeed){
    rightHangMotorSpeed = newRightHangMotorSpeed;
    
  }

  public double getLeftMotorPosition() {
    return Math.abs(leftHangMotor.getPosition()); //Return absolute value so motor inversion doesn't affect failsafes
  }

  public double getRightMotorPosition() {
    return Math.abs(rightHangMotor.getPosition()); //Return absolute value so motor inversion doesn't affect failsafes
  }

  public void resetEncoders() {
    leftHangMotor.setPosition(0);
    rightHangMotor.setPosition(0);
  }
  
  public void setSolenoids(Boolean active){
    if(active){
      leftBaseSolenoid.enable();
      rightBaseSolenoid.enable();
    } else{
      leftBaseSolenoid.disable();
      rightBaseSolenoid.disable();
    }
  }

  @Override
  public void periodic() {
    failSafes();

    leftHangMotor.set(leftHangMotorSpeed);
    rightHangMotor.set(rightHangMotorSpeed);
  }

  public void failSafes() {
    if (getLeftMotorPosition() < 0 && leftHangMotorSpeed < 0) {
      leftHangMotorSpeed = 0;
    }
    if (getRightMotorPosition() < 0 && rightHangMotorSpeed < 0) {
      rightHangMotorSpeed = 0;
    }

    if (getLeftMotorPosition() > HangConstants.maxExtension && leftHangMotorSpeed > 0) {
      leftHangMotorSpeed = 0;
    }
    if (getRightMotorPosition() > HangConstants.maxExtension && rightHangMotorSpeed > 0) {
      rightHangMotorSpeed = 0;
    }
  }
}
