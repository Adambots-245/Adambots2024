// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXMotor extends BaseMotor{

    TalonFX motor;

    public TalonFXMotor(int portNum){
        motor = new TalonFX(portNum);
    }

    public TalonFXMotor(int portNum, String bus){
        motor = new TalonFX(portNum, bus);
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
}
