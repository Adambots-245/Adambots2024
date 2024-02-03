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
  PhotoEye pieceInRobotEye;
  double groundIntakeMotorSpeed = 0;

  public IntakeSubsystem(TalonFX groundIntakeMotor, PhotoEye pieceInRobotEye){
    this.groundIntakeMotor = groundIntakeMotor;
    this.pieceInRobotEye = pieceInRobotEye;
    Dash.add("Intake Limit Switch", () -> isPieceInRobot());
    groundIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setGroundIntakeMotorSpeed(double newGroundIntakeMotorSpeed){
    groundIntakeMotorSpeed = newGroundIntakeMotorSpeed;
  }

  public boolean isPieceInRobot(){
    return pieceInRobotEye.isDetecting();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    groundIntakeMotor.set(groundIntakeMotorSpeed);
  } 
}

