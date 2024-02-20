// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.sensors.PhotoEye;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  BaseMotor groundIntakeMotor;
  PhotoEye firstPieceInRobotEye;
  PhotoEye secondPieceInRobotEye;
  PhotoEye initEye;
  double groundIntakeMotorSpeed = 0;
  boolean isIntaking = false;
  boolean isNote = false;
  boolean slowSpeed = false;

  public IntakeSubsystem(BaseMotor groundIntakeMotor, PhotoEye firstPieceInRobotEye, PhotoEye secondPieceInRobotEye){
    this.groundIntakeMotor = groundIntakeMotor;
    this.secondPieceInRobotEye = secondPieceInRobotEye;
    this.firstPieceInRobotEye = firstPieceInRobotEye;
    
    Dash.add("Second Intake Limit Switch", () -> isSecondPieceInRobot());
    Dash.add("First Intake Limit Switch", () -> isFirstPieceInRobot());
    Dash.add("Intake Velocity", () -> groundIntakeMotor.getVelocity());
    Dash.add("Intake Speed", () -> groundIntakeMotorSpeed);

    groundIntakeMotor.setNeutralMode(true);
    groundIntakeMotor.setInverted(true);
  }

  public void setGroundIntakeMotorSpeed(double newGroundIntakeMotorSpeed){
    slowSpeed = false;
    groundIntakeMotorSpeed = newGroundIntakeMotorSpeed;
  }

  public void setGroundIntakeMotorSpeedSlow(){
    groundIntakeMotorSpeed = 0.1;
    slowSpeed = true;
  }

  public boolean isSecondPieceInRobot(){
    return secondPieceInRobotEye.isDetecting();
  }

  public boolean isFirstPieceInRobot(){
    return firstPieceInRobotEye.isDetecting();
  }

  public double getIntakeSpeed(){
    return groundIntakeMotor.getVelocity()/512;
  }
  public void setNote(boolean note){
    isNote = note;
  }

  public boolean getNote(){
    return isNote;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (slowSpeed == true){
      if (groundIntakeMotor.getVelocity() < 0.5){
        groundIntakeMotor.set(groundIntakeMotorSpeed);
        groundIntakeMotorSpeed = groundIntakeMotorSpeed + 0.01; 
      } else {
        groundIntakeMotor.set(groundIntakeMotorSpeed);
        groundIntakeMotorSpeed = groundIntakeMotorSpeed - 0.01; 
      }
    } else {
      groundIntakeMotor.set(groundIntakeMotorSpeed);
    }
  } 
}

