// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.adambots.Constants;
import com.adambots.RobotMap;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.ParentDevice;
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

        var configs = new CurrentLimitsConfigs();
        if (portNum == RobotMap.shooterWheelPort) {
            configs.SupplyCurrentLimit = 45;
            configs.SupplyCurrentLimitEnable = true;
        } else {
            configs.SupplyCurrentLimitEnable = false;
        }

        motor.getConfigurator().apply(configs);

        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(50);

        ParentDevice.optimizeBusUtilizationForAll(motor);
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
        try {
            return motor.getVelocity().getValueAsDouble();
        } catch (Exception a) {
            System.out.println(a);
            return 0;
        }
    }

    // @Override
    // public double getAcceleration(){
    //     return motor.getAcceleration().getValueAsDouble();
    // }

    // @Override
    // public double getCurrentDraw(){
    //     return motor.getTorqueCurrent().getValueAsDouble();
    // }

    // @Override
    // public boolean getForwardLimitSwitch() {
    //     return motor.getForwardLimit().getValueAsDouble() == 1;
    // }

    // @Override
    // public boolean getReverseLimitSwitch() {
    //     return motor.getReverseLimit().getValueAsDouble() == 1;
    // }
}
