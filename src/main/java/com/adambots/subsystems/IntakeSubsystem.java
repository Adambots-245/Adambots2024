// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.sensors.DigitalSensor;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private BaseMotor intakeMotor;

  private DigitalSensor firstPieceInRobotEye;
  private DigitalSensor secondPieceInRobotEye;
  
  private double motorSpeed = 0;
  private boolean distanceMode = false;
  private double targetDist = 0;
  private boolean lockOut = false;

  public IntakeSubsystem(BaseMotor intakeMotor, DigitalSensor firstPieceInRobotEye, DigitalSensor secondPieceInRobotEye){
    this.intakeMotor = intakeMotor;
    this.secondPieceInRobotEye = secondPieceInRobotEye;
    this.firstPieceInRobotEye = firstPieceInRobotEye;
    
    Dash.add("Second Intake Limit Switch", () -> isSecondPieceInRobot());
    Dash.add("First Intake Limit Switch", () -> isFirstPieceInRobot());
    Dash.add("Intake Velocity", () -> intakeMotor.getVelocity());
    Dash.add("Intake Speed", () -> motorSpeed);

    intakeMotor.setBrakeMode(true);
    intakeMotor.setInverted(true);
  }

  public void setMotorSpeed(double newMotorSpeed){
    motorSpeed = newMotorSpeed;
  }

  public boolean isSecondPieceInRobot(){
    return secondPieceInRobotEye.isDetecting();
  }

  public boolean isFirstPieceInRobot(){
    return firstPieceInRobotEye.isDetecting();
  }

  public void moveDistance(double dist) {
    distanceMode = true;
    targetDist = dist;
    intakeMotor.setPosition(0);
  }

  public double getIntakeSpeed(){
    return intakeMotor.getVelocity();
  }

  public boolean getLockOut() {
    return lockOut;
  }

  public void setLockOut(boolean bool) {
    lockOut = bool;
  }

  @Override
  public void periodic() {
    if (!distanceMode) {
      intakeMotor.set(motorSpeed);
    } else {
      motorSpeed = 0;

      double error = targetDist-intakeMotor.getPosition();
      double speed = MathUtil.clamp(error, -0.1, 0.1);
      intakeMotor.set(speed);

      if (Math.abs(error) < 0.1) {
        distanceMode = false;
      }
    }
  } 
}

