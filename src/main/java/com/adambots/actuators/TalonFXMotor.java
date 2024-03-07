// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.adambots.Constants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXMotor implements BaseMotor{

    TalonFX motor;

    public TalonFXMotor(int portNum, Boolean isOnCANivore){
        if (isOnCANivore) {
            motor = new TalonFX(portNum, "*"); //'*' will assign the motor on any CANivore seen by the program
        } else {
            motor = new TalonFX(portNum);
        }

        motor.getVelocity().setUpdateFrequency(50); //Any status signals must be listed here in order for their value to update
        motor.getPosition().setUpdateFrequency(50);
        motor.getForwardLimit().setUpdateFrequency(25);
        motor.getReverseLimit().setUpdateFrequency(25);

        ParentDevice.optimizeBusUtilizationForAll(motor); //Set update frequency for any unused status objects to 0
    }

    public TalonFXMotor(int portNum, Boolean isOnCANivore, double currentLimitAmps){
        new TalonFXMotor(portNum, isOnCANivore);

        var configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimit = currentLimitAmps;
        configs.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(configs);
    }

    @Override
    public void set(double speed){
        motor.set(speed);
    }

    @Override
    public void setInverted(boolean inverted){
        motor.setInverted(inverted);
    }

    @Override
    public void setNeutralMode(boolean brake){
        if(brake){
           motor.setNeutralMode(NeutralModeValue.Brake);
        }else{
            motor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void setPosition(double rotations){
        motor.setPosition(rotations);
    }

    @Override
    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimit().getValueAsDouble() == 1;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimit().getValueAsDouble() == 1;
    }

    @Override
    public void setSmartCurrentLimit(int value) {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimit = value;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(currentLimitsConfigs);
    }

    @Override
    public void enableVoltageCompensation(double value) {
        
        final VoltageOut m_request = new VoltageOut(0);
        motor.setControl(m_request.withOutput(value));
    }

    @Override
    public double getAcceleration() {

        return motor.getAcceleration().getValueAsDouble();
    }

    @Override
    public double getCurrentDraw() {

        return motor.getStatorCurrent().getValueAsDouble();
    }
}
