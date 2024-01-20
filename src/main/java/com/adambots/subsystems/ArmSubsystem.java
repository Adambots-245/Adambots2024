// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX shoulderMotor;
  TalonFX wristMotor;
  CANCoder shoulderEncoder;
  CANCoder wristEncoder;
  PIDController shoulderPID= new PIDController(0, 0, 0);
  PIDController wristPID= new PIDController(0, 0, 0);

  double shoulderAngle = 0;
  double wristAngle = 0;
  double shoulderSpeed, wristSpeed = 0;

  public ArmSubsystem(TalonFX shoulderMotor, TalonFX wristMotor, CANCoder shoulderEncoder, CANCoder wristEncoder) {
    this.shoulderMotor=shoulderMotor;
    this.wristMotor=wristMotor;
    this.shoulderEncoder=shoulderEncoder;
    this.wristEncoder=wristEncoder;
  }

  public void setShoulderAngle(double newShoulderAngle) {
    shoulderAngle = newShoulderAngle;
  }

  public void setWristAngle(double newWristAngle) {
    wristAngle = newWristAngle;
  }

  @Override
  public void periodic() {
    shoulderSpeed = shoulderPID.calculate(shoulderEncoder.getAbsolutePosition(), shoulderAngle);
    wristSpeed = wristPID.calculate(wristEncoder.getAbsolutePosition(), wristAngle);
    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);
  }
}
