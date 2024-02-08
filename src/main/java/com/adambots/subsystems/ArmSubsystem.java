// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX shoulderMotor;
  TalonFX wristMotor;
  DigitalInput shoulderUpperLimit;
  DigitalInput shoulderLowerLimit;
  DigitalInput wristUpperLimit;
  DigitalInput wristLowerLimit;
  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder wristEncoder;
  PIDController shoulderPID = new PIDController(0.2, 0, 0.01);
  PIDController wristPID = new PIDController(0.2, 0, 0.01);

  double shoulderAngle = 0;
  double wristAngle = 0;
  double shoulderSpeed, wristSpeed = 0;

  public ArmSubsystem(TalonFX shoulderMotor, TalonFX wristMotor, DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder, DigitalInput shoulderLowerLimit, DigitalInput wristLowerLimit) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;
    this.shoulderLowerLimit = shoulderLowerLimit;
    this.wristLowerLimit = wristLowerLimit;
  }

  public void setShoulderAngle(double newShoulderAngle) {
    shoulderAngle = newShoulderAngle;
  }

  public void setWristAngle(double newWristAngle) {
    wristAngle = newWristAngle;
  }

  public boolean shoulderLowerLimitPressed() {
    return shoulderLowerLimit.get();
  }

  public boolean wristLowerLimitPressed() {
    return wristLowerLimit.get();
  }

  public void incrementShoulderAngle(double shoulderIncrement) {
    shoulderAngle += shoulderIncrement;

  }
    public void incrementWristAngle(double wristIncrement) {
    wristAngle += wristIncrement;
  }
  
  @Override
  public void periodic() {
    shoulderSpeed = shoulderPID.calculate(shoulderEncoder.getAbsolutePosition(), shoulderAngle);
    wristSpeed = wristPID.calculate(wristEncoder.getAbsolutePosition(), wristAngle);
    failSafes();
    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);

  }

  private void failSafes() {
    if (shoulderLowerLimitPressed()) {
      shoulderEncoder.reset();
      if (shoulderSpeed < 0) {
        shoulderSpeed = 0;
      }
    }
    if (wristLowerLimitPressed()){
      wristEncoder.reset();
      if (wristSpeed < 0) {
        wristSpeed = 0;
      }
    }

  }
}
