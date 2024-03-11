// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ArmConstants.StateName;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private BaseMotor shoulderMotor;
  private BaseMotor wristMotor;
  private BaseAbsoluteEncoder shoulderEncoder;
  private BaseAbsoluteEncoder wristEncoder;
  private PIDController shoulderPID = new PIDController(0.02, 0.008, 0.0028); //0.018, 0.008, 0.002
  private PIDController wristPID = new PIDController(0.0062, 0.009, 0.00062); //0.0061, 0.009, 0.00062

  private double shoulderLowerLimit = ArmConstants.shoulderLowerLimit;
  private double shoulderUpperLimit = ArmConstants.shoulderUpperLimit;
  private double wristLowerLimit = ArmConstants.wristLowerLimit;
  private double wristUpperLimit = ArmConstants.wristUpperLimit;

  private double targetShoulderAngle;
  private double targetWristAngle;
  private double shoulderSpeed, wristSpeed = 0;

  private double shoulderAngleOffset, wristAngleOffset = 0;

  private State currentState = ArmConstants.defaultState;

  private boolean failsafeOverride = false;

  public ArmSubsystem(BaseMotor shoulderMotor, BaseMotor wristMotor, BaseAbsoluteEncoder shoulderEncoder, BaseAbsoluteEncoder wristEncoder) {
    this.shoulderMotor = shoulderMotor;
    this.wristMotor = wristMotor;
    this.shoulderEncoder = shoulderEncoder;
    this.wristEncoder = wristEncoder;

    shoulderMotor.setInverted(false);
    wristMotor.setInverted(false);

    shoulderMotor.setBrakeMode(true);
    wristMotor.setBrakeMode(true);

    shoulderPID.enableContinuousInput(0, 360);
    wristPID.enableContinuousInput(0, 360);

    shoulderPID.setIntegratorRange(-0.025, 0.025);
    wristPID.setIntegratorRange(-0.02, 0.02);

    shoulderPID.setTolerance(1);
    wristPID.setTolerance(1);

    shoulderAngleOffset = this.shoulderEncoder.getAbsolutePositionDegrees();
    wristAngleOffset = this.wristEncoder.getAbsolutePositionDegrees();

    shoulderMotor.setPosition(0);
    wristMotor.setPosition(0);

    targetShoulderAngle = getCurrentShoulderAngle();
    targetWristAngle = getCurrentWristAngle();

    Dash.add("Shoulder Encoder", () -> getCurrentShoulderAngle());
    Dash.add("Wrist Encoder", () -> getCurrentWristAngle());
    Dash.add("Shoulder Motor Encoder", () -> shoulderMotor.getPosition()*ArmConstants.kShoulderEncoderPositionConversionFactor + shoulderAngleOffset);
    Dash.add("Wrist Motor Encoder", () -> wristMotor.getPosition()*ArmConstants.kWristEncoderPositionConversionFactor + wristAngleOffset);
    Dash.add("wristSpeed", () ->  wristSpeed);
    Dash.add("shoulderSpeed", () ->  shoulderSpeed);

    // Dash.add("Shld Fwd Lim", () -> shoulderMotor.getForwardLimitSwitch());
    // Dash.add("Shld Rev Lim", () -> shoulderMotor.getReverseLimitSwitch());

    // Dash.add("Wrst Fwd Lim", () -> wristMotor.getForwardLimitSwitch());
    // Dash.add("Wrst Rev Lim", () -> wristMotor.getReverseLimitSwitch());
  }

  private void setPidTolerence(State state) {
    shoulderPID.setTolerance(state.getShoulderTolerance());
    wristPID.setTolerance(state.getWristTolerance());
  }

  public void incrementShoulderAngle(double shoulderIncrement) {
    targetShoulderAngle += shoulderIncrement;
    failsafeOverride = false;

    currentState = new State(targetWristAngle, targetShoulderAngle, StateName.CUSTOM);
  }

  public void incrementWristAngle(double wristIncrement) {
    targetWristAngle += wristIncrement;
    failsafeOverride = false;

    currentState = new State(targetWristAngle, targetShoulderAngle, StateName.CUSTOM);
  }

  public boolean isAtTargetState () {
    return (Math.abs(shoulderPID.getPositionError()) < 10 && Math.abs(wristPID.getPositionError()) < 10); 
  }

  public double getCurrentWristAngle(){
    return wristMotor.getPosition()*ArmConstants.kWristEncoderPositionConversionFactor + wristAngleOffset;
    // return wristEncoder.getAbsolutePositionDegrees();
  }

  public double getCurrentShoulderAngle(){
    return shoulderMotor.getPosition()*ArmConstants.kShoulderEncoderPositionConversionFactor + shoulderAngleOffset;
    // return shoulderEncoder.getAbsolutePositionDegrees();
  }

  public void setCurrentState(State newState) {
    //Enable override if we are moving directly from speaker to floor states (auton) or if we are targeting the hang state
    failsafeOverride = (currentState.getStateName() == StateName.SPEAKER && newState.getStateName() == StateName.FLOOR) || newState.getStateName() == StateName.HANG;

    currentState = newState;
    targetShoulderAngle = newState.getShoulderAngle();
    targetWristAngle = newState.getWristAngle();

    setPidTolerence(currentState);

    wristPID.reset();
    shoulderPID.reset();
  }

  public State getCurrentState() {
    return currentState;
  }

  public StateName getCurrentStateName() {
    return currentState.getStateName();
  }
  
  @Override
  public void periodic() {
    System.out.println(wristMotor.getPosition());

    if (DriverStation.isEnabled()){
      shoulderSpeed = shoulderPID.calculate(getCurrentShoulderAngle(), targetShoulderAngle);
      wristSpeed = wristPID.calculate(getCurrentWristAngle(), targetWristAngle);
    } else {
      wristSpeed = 0;
      shoulderSpeed = 0;
    }

    failSafes();

    if(failsafeOverride){
     shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeedNitro, ArmConstants.maxShoulderUpSpeed);
    }else{
      shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeed, ArmConstants.maxShoulderUpSpeed);
    }
    
    wristSpeed = MathUtil.clamp(wristSpeed, -ArmConstants.maxWristSpeed, ArmConstants.maxWristSpeed);



    // shoulderSpeed = MathUtil.clamp(shoulderSpeed, -0.4, 0.4);
    // wristSpeed = MathUtil.clamp(wristSpeed, -0.1, 0.1);


    
    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);
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
