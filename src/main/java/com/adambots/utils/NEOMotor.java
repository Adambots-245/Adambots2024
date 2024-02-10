// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class NEOMotor extends Motor{

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
    public void setNeutralMode(boolean brake){
        if(brake){
            motor.setIdleMode(IdleMode.kBrake);
        }else{
            motor.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public double getPosition(){
        System.out.println("WARNING: Functionality for this method has not been implemented.");
        return 0;
    }

    @Override
    public double getVelocity(){
        System.out.println("WARNING: Functionality for this method has not been implemented.");
        return 0;
    }

    @Override
    public double getAcceleration(){
        System.out.println("WARNING: Functionality for this method has not been implemented.");
        return 0;
    }

}
