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
  private PIDController shoulderPID = new PIDController(0.02, 0.01, 0.0028); //0.02, 0.008, 0.0028
  private PIDController wristPID = new PIDController(0.0062, 0.009, 0.00062); //0.0061, 0.009, 0.00062

  private double shoulderLowerLimit = ArmConstants.shoulderLowerLimit;
  private double shoulderUpperLimit = ArmConstants.shoulderUpperLimit;
  private double wristLowerLimit = ArmConstants.wristLowerLimit;
  private double wristUpperLimit = ArmConstants.wristUpperLimit;

  private double targetShoulderAngle;
  private double targetWristAngle;
  private double shoulderSpeed, wristSpeed = 0;

  // private int speedDebounce = 0;

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

    targetShoulderAngle = this.shoulderEncoder.getAbsolutePositionDegrees();
    targetWristAngle = this.wristEncoder.getAbsolutePositionDegrees();

    Dash.add("Shoulder Encoder", () -> shoulderEncoder.getAbsolutePositionDegrees());
    Dash.add("Wrist Encoder", () -> wristEncoder.getAbsolutePositionDegrees());
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
    return (Math.abs(shoulderPID.getPositionError()) < 5 && Math.abs(wristPID.getPositionError()) < 5); 
  }

  public double getCurrentWristShaftAngle(){
    return wristEncoder.getAbsolutePositionDegrees();
  }

  public double getCurrentShoulderShaftAngle(){
    return shoulderEncoder.getAbsolutePositionDegrees();
  }

  public double getCurrentWristMotorAngle(){
    return wristMotor.getPosition()*ArmConstants.kWristEncoderPositionConversionFactor + wristAngleOffset;
  }

  public double getCurrentShoulderMotorAngle(){
    return shoulderMotor.getPosition()*ArmConstants.kShoulderEncoderPositionConversionFactor + shoulderAngleOffset;
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
    if (DriverStation.isEnabled()){
      shoulderSpeed = shoulderPID.calculate(getCurrentShoulderMotorAngle(), targetShoulderAngle);
      if (currentState.getStateName() == StateName.FLOOR) {
        shoulderSpeed = shoulderSpeed - 0.3;
      }
      wristSpeed = wristPID.calculate(getCurrentWristShaftAngle(), targetWristAngle);
    } else {
      wristSpeed = 0;
      shoulderSpeed = 0;
    }

    if(currentState.getStateName() == StateName.FLOOR && getCurrentShoulderShaftAngle() < ArmConstants.floorShoulderAngle + 2) {
      System.out.println("RESET");
      wristAngleOffset = wristEncoder.getAbsolutePositionDegrees();
      wristMotor.setPosition(0);
    }

    // if (Math.abs(wristEncoder.getAbsolutePositionDegrees() - getCurrentWristAngle()) > 5) {
    //   System.out.println("RESET");
    //   wristAngleOffset = wristEncoder.getAbsolutePositionDegrees();
    //   wristMotor.setPosition(0);
    // }
    // if (Math.abs(wristSpeed) < 0.1 && getCurrentStateName() == StateName.DEFAULT) {
    //   speedDebounce++;
    // } else {
    //   speedDebounce--;
    // }
    // speedDebounce = MathUtil.clamp(speedDebounce, 0, 50);
    // if (speedDebounce >= 50) {
    //   System.out.println("RESET");
    //   speedDebounce = 0;
    //   wristAngleOffset = wristEncoder.getAbsolutePositionDegrees();
    //   wristMotor.setPosition(0);
    // }

    failSafes();

    if(failsafeOverride){
     shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeedNitro, ArmConstants.maxShoulderUpSpeed);
    }else{
      shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.maxShoulderDownSpeed, ArmConstants.maxShoulderUpSpeed);
    }
    
    wristSpeed = MathUtil.clamp(wristSpeed, -ArmConstants.maxWristSpeed, ArmConstants.maxWristSpeed);
    
    shoulderMotor.set(shoulderSpeed);
    wristMotor.set(wristSpeed);
  }

  private void failSafes() {
    if(getCurrentShoulderShaftAngle() < ArmConstants.shoulderDangerZoneThreshold && !failsafeOverride){
      wristLowerLimit = ArmConstants.wristDangerZoneLowerLimit;
    }else{
      wristLowerLimit = ArmConstants.wristLowerLimit;
    }

    if(!failsafeOverride && getCurrentShoulderShaftAngle() < ArmConstants.shoulderDangerZoneThreshold && getCurrentWristShaftAngle() < ArmConstants.wristShoulderStopLimit && shoulderSpeed < 0){
      shoulderSpeed = 0;
    }

    if (getCurrentShoulderShaftAngle() > shoulderUpperLimit && shoulderSpeed > 0){
      shoulderSpeed = 0;
    } 
    if(getCurrentShoulderShaftAngle() < shoulderLowerLimit && shoulderSpeed < 0) {
      shoulderSpeed = 0;
    }

    if (getCurrentWristShaftAngle() < wristLowerLimit && wristSpeed < 0){
      wristSpeed = 0;
    }
    if(getCurrentWristShaftAngle() > wristUpperLimit && wristSpeed > 0){
      wristSpeed = 0;
    }

    if (getCurrentShoulderShaftAngle() == 0) {
      shoulderSpeed = 0;
      System.err.println("WARNING: SHOULDER ENCODER DISCONNECTED");
    }
    if (getCurrentWristShaftAngle() == 0) {
      wristSpeed = 0;
      System.err.println("WARNING: WRIST ENCODER DISCONNECTED");
    }
  }
}
