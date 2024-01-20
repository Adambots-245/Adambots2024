// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.sensors.PhotoEye;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax groundIntakeMotor;
  PhotoEye pieceInRobotEye;
  double groundIntakeMotorSpeed = 0;

  public IntakeSubsystem(CANSparkMax groundIntakeMotor, PhotoEye pieceInRobotEye){
    this.groundIntakeMotor = groundIntakeMotor;
    this.pieceInRobotEye = pieceInRobotEye;
  }

  public void setGroundIntakeMotorSpeed(double newGroundIntakeMotorSpeed){
    groundIntakeMotorSpeed = newGroundIntakeMotorSpeed;
  }

  public boolean isPieceInRobot(){
    return pieceInRobotEye.isDetecting();
  }
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(isPieceInRobot() == true){
       setGroundIntakeMotorSpeed(0);
        
    }
    groundIntakeMotor.set(groundIntakeMotorSpeed);
  } 
}

