// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.HangConstants;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  
  private BaseMotor leftHangMotor;
  private BaseMotor rightHangMotor;
  private double leftHangMotorSpeed, rightHangMotorSpeed = 0;
  private Relay leftRelay;
  private Relay rightRelay;

  public HangSubsystem(BaseMotor leftHangMotor, BaseMotor rightHangMotor, Relay rightRelay, Relay leftRelay) {
    this.leftHangMotor = leftHangMotor;
    this.rightHangMotor = rightHangMotor;
    this.leftRelay = leftRelay;
    this.rightRelay = rightRelay;

    leftHangMotor.setInverted(true);
    rightHangMotor.setInverted(false);

    leftHangMotor.setBrakeMode(true);
    rightHangMotor.setBrakeMode(true);

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
      leftRelay.set(Value.kOn);
      rightRelay.set(Value.kOn);
    } else{
      leftRelay.set(Value.kOff);
      rightRelay.set(Value.kOff);
    }
  }

  @Override
  public void periodic() {
    failSafes();

    // System.out.println(leftHangMotor.getVelocity());

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
