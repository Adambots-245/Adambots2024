// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  
  TalonFX leftHangMotor;
  TalonFX rightHangMotor;
  double leftHangMotorSpeed;
  double rightHangMotorSpeed;
  DigitalInput leftHangLimit;
  DigitalInput rightHangLimit;

  public HangSubsystem(TalonFX leftHangMotor, TalonFX rightHangMotor, DigitalInput leftHangLimit, DigitalInput rightHangLimit) {
    this.leftHangMotor = leftHangMotor;
    this.rightHangMotor = rightHangMotor;
    this.leftHangLimit = leftHangLimit;
    this.rightHangLimit = rightHangLimit;
  }

  public void setLeftHangMotorSpeed(double newLeftHangMotorSpeed){
    leftHangMotorSpeed = newLeftHangMotorSpeed;
  }

  public void setRightHangMotorSpeed(double newRightHangMotorSpeed){
    rightHangMotorSpeed = newRightHangMotorSpeed;
  }

  public boolean isLeftHangRetracted(){
    return leftHangLimit.get();
  }

  public boolean isRightHangRetracted(){
    return rightHangLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftHangMotor.set(leftHangMotorSpeed);
    rightHangMotor.set(rightHangMotorSpeed);

    if(isLeftHangRetracted() == true){
      leftHangMotor.setPosition(0);
    }
    if(isRightHangRetracted() == true){
      rightHangMotor.setPosition(0);
    }
  }
}
