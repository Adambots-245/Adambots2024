// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class NEOMotor extends BaseMotor{

    CANSparkMax motor;

    public NEOMotor(int portNum, boolean brushed){
        if(brushed){
            motor = new CANSparkMax(portNum, MotorType.kBrushed);
        }else{
           motor = new CANSparkMax(portNum, MotorType.kBrushless);
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
    public void setBrakeMode(boolean brake){
        if(brake){
            motor.setIdleMode(IdleMode.kBrake);
        }else{
            motor.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public double getCurrentDraw(){
        return motor.getOutputCurrent();
    }
}
