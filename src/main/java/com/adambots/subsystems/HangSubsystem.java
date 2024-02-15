// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  
  TalonFX leftHangMotor;
  TalonFX rightHangMotor;
  double leftHangMotorSpeed;
  double rightHangMotorSpeed;
  Relay leftRelay;
  Relay rightRelay;
  boolean solenoidLock;

  // Add Single Solenoid, will be attached to RelayPort, need to look at previous years code to know how to point to Relay port

  public HangSubsystem(TalonFX leftHangMotor, TalonFX rightHangMotor, Relay leftRelay, Relay rightRelay) {
    this.leftHangMotor = leftHangMotor;
    this.rightHangMotor = rightHangMotor;
    this.leftRelay = leftRelay;
    this.rightRelay = rightRelay;
  }

  public void setLeftHangMotorSpeed(double newLeftHangMotorSpeed){
    leftHangMotorSpeed = newLeftHangMotorSpeed;
  }

  public void setRightHangMotorSpeed(double newRightHangMotorSpeed){
    rightHangMotorSpeed = newRightHangMotorSpeed;
  }

  public Value isLeftHangRetracted(){
    return leftRelay.get();
  }

  public Value isRightHangRetracted(){
    return rightRelay.get();
  }
  
  public void setRelay(boolean lock){
      solenoidLock = lock;
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftHangMotor.set(leftHangMotorSpeed);
    rightHangMotor.set(rightHangMotorSpeed);

    if(isLeftHangRetracted() == Value.kOff){
      leftHangMotor.setPosition(0);
    }
    if(isRightHangRetracted() == Value.kOff){
      rightHangMotor.setPosition(0);
    }

    if(solenoidLock){
      leftRelay.set(Value.kOn);
      rightRelay.set(Value.kOn);
    }
    else{
      leftRelay.set(Value.kOff);
      rightRelay.set(Value.kOff);
    }
    
  }
}
