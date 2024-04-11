// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseProximitySensor;
import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private BaseMotor intakeMotor;
  private BaseProximitySensor firstPieceInRobotEye;
  private BaseProximitySensor secondPieceInRobotEye;

  private double motorSpeed = 0;
  private boolean lockOut = false;

  public IntakeSubsystem(BaseMotor intakeMotor, BaseProximitySensor firstPieceInRobotEye, BaseProximitySensor secondPieceInRobotEye) {
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

  public void setMotorSpeed(double newMotorSpeed) {
    motorSpeed = newMotorSpeed;
  }

  public boolean isSecondPieceInRobot() {
    return secondPieceInRobotEye.isDetecting();
  }

  public boolean isFirstPieceInRobot() {
    return firstPieceInRobotEye.isDetecting();
  }

  public boolean getLockOut() {
    return lockOut;
  }

  public void setLockOut(boolean bool) {
    lockOut = bool;
  }

  @Override
  public void periodic() {
    intakeMotor.set(motorSpeed);
  }
}
