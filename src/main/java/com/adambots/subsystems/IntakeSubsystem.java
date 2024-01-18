// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.adambots.DigitalInput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  TalonFX groundIntakeMotor;
  TalonFX middleConveyor;
  TalonFX rotateIntakeMotor;
  DigitalInput lowerIntakeLimit;
  DigitalInput upperIntakeLimit;
  PhotoEye pieceDetectionEye;
  PhotoEye pieceInRobotEye;

  double groundIntakeMotorSpeed = 0;
  double middleConveyorSpeed = 0;
  double rotateIntakeMotorSpeed = 0;

  public IntakeSubsystem(TalonFX groundIntakeMotor, TalonFX middleConveyor, TalonFX rotateIntakeMotor, DigitalInput lowerIntakeLimit, DigitalInput upperIntakeLimit, PhotoEye pieceDetectionEye, PhotoEye pieceInRobotEye){
    this.groundIntakeMotor = groundIntakeMotor;
    this.middleConveyor = middleConveyor;
    this.rotateIntakeMotor = rotateIntakeMotor;
    this.lowerIntakeLimit = lowerIntakeLimit;
    this.upperIntakeLimit = upperIntakeLimit;
    this.pieceDetectionEye = pieceDetectionEye;
    this.pieceInRobotEye = pieceInRobotEye;
  }

  public void setGroundIntakeMotorSpeed(double newGroundIntakeMotorSpeed){
    groundIntakeMotorSpeed = newGroundIntakeMotorSpeed;
  }

  public boolean isIntakeUp(DigitalInput upperIntakeLimit){
    return upperIntakeLimit.isDetecting;
  }
  
  public void setMiddleConveyorSpeed(double newMiddleConveyorSpeed){
    middleConveyorSpeed = newMiddleConveyorSpeed;
  }

  public void setRotateIntakeMotorSpeed(double newRotateIntakeMotorSpeed){
    rotateIntakeMotorSpeed = newRotateIntakeMotorSpeed;
  }
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    middleConveyor.set(middleConveyorSpeed);
    groundIntakeMotor.set(groundIntakeMotorSpeed);
    rotateIntakeMotor.set(rotateIntakeMotorSpeed);
  }
}
