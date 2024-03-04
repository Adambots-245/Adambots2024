// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.sensors.AbsoluteEncoder;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private BaseMotor shoulderMotor;
  private BaseMotor wristMotor;
  private AbsoluteEncoder shoulderEncoder;
  private AbsoluteEncoder wristEncoder;
  private PIDController shoulderPID = new PIDController(0.02, 0.008, 0.0028); //0.018, 0.008, 0.002
  private PIDController wristPID = new PIDController(0.0062, 0.009, 0.00062); //0.0061, 0.009, 0.00062

  private double shoulderLowerLimit = ArmConstants.shoulderLowerLimit;
  private double shoulderUpperLimit = ArmConstants.shoulderUpperLimit;
  private double wristLowerLimit = ArmConstants.wristLowerLimit;
  private double wristUpperLimit = ArmConstants.wristUpperLimit;

  private double targetShoulderAngle;
  private double targetWristAngle;
  private double shoulderSpeed, wristSpeed = 0;

  private String currentStateName = "default";

  private boolean failsafeOverride = false;

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, AbsoluteEncoder shoulderEncoder, AbsoluteEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    targetShoulderAngle = getCurrentShoulderAngle();
    targetWristAngle = getCurrentWristAngle();

    shoulderMotor.setInverted(false);
    wristMotor.setInverted(false);

    shoulderMotor.setBrakeMode(true);
    wristMotor.setBrakeMode(true);

    shoulderPID.enableContinuousInput(0, 360);
    wristPID.enableContinuousInput(0, 360);

    shoulderPID.setIntegratorRange(-0.025, 0.025);
    wristPID.setIntegratorRange(-0.02, 0.02);

    setPidTolerence(1);

    Dash.add("Shoulder Encoder", () -> getCurrentShoulderAngle());
    Dash.add("Wrist Encoder", () -> getCurrentWristAngle());
    Dash.add("wristSpeed", () ->  wristSpeed);
    Dash.add("shoulderSpeed", () ->  shoulderSpeed);

    // Dash.add("Shld Fwd Lim", () -> shoulderMotor.getForwardLimitSwitch());
    // Dash.add("Shld Rev Lim", () -> shoulderMotor.getReverseLimitSwitch());

    // Dash.add("Wrst Fwd Lim", () -> wristMotor.getForwardLimitSwitch());
    // Dash.add("Wrst Rev Lim", () -> wristMotor.getReverseLimitSwitch());
  }

  private void setPidTolerence(double degreesOfTolerence) {
    shoulderPID.setTolerance(degreesOfTolerence);
    wristPID.setTolerance(degreesOfTolerence);
  }

  public void incrementShoulderAngle(double shoulderIncrement) {
    targetShoulderAngle += shoulderIncrement;
    failsafeOverride = false;
  }

  public void incrementWristAngle(double wristIncrement) {
    targetWristAngle += wristIncrement;
    failsafeOverride = false;
  }

  public boolean isAtTargetState () {
    return (Math.abs(shoulderPID.getPositionError()) < 3 && Math.abs(wristPID.getPositionError()) < 3); 
  }

  public double getCurrentWristAngle(){
    return wristEncoder.getAbsolutePositionDegrees();
  }

  public double getCurrentShoulderAngle(){
    return shoulderEncoder.getAbsolutePositionDegrees();
  }

  public void setCurrentState(State newState) {
    //Enable override if we are moving directly from speaker to floor states (auton) or if we are targeting the hang state
    failsafeOverride = ((currentStateName.equals("speaker") && newState.getStateName().equals("floor")) || newState.getStateName().equals("hang"));

    currentStateName = newState.getStateName();
    targetShoulderAngle = newState.getShoulderAngle();
    targetWristAngle = newState.getWristAngle();

    //Dynamically change pid tolerences depending on the arm state
    if (currentStateName.equals("speaker")) {
      shoulderPID.setTolerance(2);
      wristPID.setTolerance(1);
    } else if (currentStateName.equals("floor")) {
      setPidTolerence(0);
    } else if (currentStateName.equals("amp")) {
      shoulderPID.setTolerance(1.5);
      wristPID.setTolerance(5);
    } else if (currentStateName.equals("human")) {
      setPidTolerence(2);
    } else if (currentStateName.equals("default")) {
      setPidTolerence(5);
    }else {
      setPidTolerence(0);
    }

    wristPID.reset();
    shoulderPID.reset();
  }

  public String getCurrentStateName() {
    return currentStateName;
  }
  
  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      shoulderSpeed = shoulderPID.calculate(getCurrentShoulderAngle(), targetShoulderAngle);
      wristSpeed = wristPID.calculate(getCurrentWristAngle(), targetWristAngle);
    }
    else {
      shoulderSpeed = 0;
      wristSpeed = 0;
    }

    failSafes();

    if(failsafeOverride){
     shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeedNitro, ArmConstants.maxShoulderUpSpeed);
    }else{
      shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeed, ArmConstants.maxShoulderUpSpeed);
    }
    wristSpeed = MathUtil.clamp(wristSpeed, -ArmConstants.maxWristSpeed, ArmConstants.maxWristSpeed);


    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);
    // shoulderMotor.set(0);
    // wristMotor.set(0);
  }

  private void failSafes() {
    if(getCurrentShoulderAngle() < ArmConstants.shoulderDangerZoneThreshold && !failsafeOverride){
      wristLowerLimit = ArmConstants.wristDangerZoneLowerLimit;
    }else{
      wristLowerLimit = ArmConstants.wristLowerLimit;
    }

    if(!failsafeOverride && getCurrentShoulderAngle() < ArmConstants.shoulderDangerZoneThreshold && getCurrentWristAngle() < ArmConstants.wristShoulderStopLimit && shoulderSpeed < 0){
      shoulderSpeed = 0;
    }

    if (getCurrentShoulderAngle() > shoulderUpperLimit && shoulderSpeed > 0){
      shoulderSpeed = 0;
    } 
    if(getCurrentShoulderAngle() < shoulderLowerLimit && shoulderSpeed < 0) {
      shoulderSpeed = 0;
    }

    if (getCurrentWristAngle() < wristLowerLimit && wristSpeed < 0){
      wristSpeed = 0;
    }
    if(getCurrentWristAngle() > wristUpperLimit && wristSpeed > 0){
      wristSpeed = 0;
    }

    if (getCurrentShoulderAngle() == 0) {
      shoulderSpeed = 0;
      System.err.println("WARNING: SHOULDER ENCODER DISCONNECTED");
    }
    if (getCurrentWristAngle() == 0) {
      wristSpeed = 0;
      System.err.println("WARNING: WRIST ENCODER DISCONNECTED");
    }
  }
}
