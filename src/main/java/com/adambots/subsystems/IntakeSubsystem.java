// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.sensors.PhotoEye;
import com.adambots.utils.Dash;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {

  TalonFX groundIntakeMotor;
  PhotoEye firstPieceInRobotEye;
  PhotoEye secondPieceInRobotEye;
  PhotoEye initEye;
  double groundIntakeMotorSpeed = 0;
  boolean isIntaking = false;

  public IntakeSubsystem(TalonFX groundIntakeMotor, PhotoEye firstPieceInRobotEye, PhotoEye secondPieceInRobotEye){
    this.groundIntakeMotor = groundIntakeMotor;
    this.secondPieceInRobotEye = secondPieceInRobotEye;
    this.firstPieceInRobotEye = firstPieceInRobotEye;
    Dash.add("Second Intake Limit Switch", () -> isSecondPieceInRobot());
    Dash.add("First Intake Limit Switch", () -> isFirstPieceInRobot());

    groundIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    groundIntakeMotor.setInverted(true);
  }

  public void setGroundIntakeMotorSpeed(double newGroundIntakeMotorSpeed){
    groundIntakeMotorSpeed = newGroundIntakeMotorSpeed;
  }

  public boolean isSecondPieceInRobot(){
    return secondPieceInRobotEye.isDetecting();
  }

  public boolean isFirstPieceInRobot(){
    return firstPieceInRobotEye.isDetecting();
  }

  public double getIntakeSpeed(){
    return groundIntakeMotor.getVelocity().getValueAsDouble()/512;
  }
  public boolean getIntake(){
    if (getIntakeSpeed() > 0 && groundIntakeMotor.getSupplyCurrent().getValueAsDouble() > 3){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    groundIntakeMotor.set(groundIntakeMotorSpeed);
  } 
}

