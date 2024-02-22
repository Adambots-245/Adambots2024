// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.adambots.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXMotor extends BaseMotor{

    TalonFX motor;

    public TalonFXMotor(int portNum, Boolean isOnCANivore){
        if (isOnCANivore) {
            motor = new TalonFX(portNum, Constants.CANivoreBus);
        } else {
            motor = new TalonFX(portNum);
        }
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
    public double getAcceleration(){
        return motor.getAcceleration().getValueAsDouble();
    }

    @Override
    public double getCurrentDraw(){
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimit().getValueAsDouble() == 1;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimit().getValueAsDouble() == 1;
    }
}
