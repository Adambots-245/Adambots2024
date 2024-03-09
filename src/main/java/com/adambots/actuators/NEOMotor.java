// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class NEOMotor implements BaseMotor{

    CANSparkMax motor;

    public NEOMotor(int portNum, boolean brushed){
        motor = new CANSparkMax(portNum, brushed ? MotorType.kBrushed : MotorType.kBrushless);
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
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getVelocity(){
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getAcceleration(){
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getCurrentDraw(){
        return motor.getOutputCurrent();
    }

    @Override
    public void setPosition(double rotations) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public boolean getForwardLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getForwardLimitSwitch'");
    }

    @Override
    public boolean getReverseLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getReverseLimitSwitch'");
    }

    @Override
    public void enableVoltageCompensation(double value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableVoltageCompensation'");
    }
}
