// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

/** Add your docs here. */
public class SparkMax implements BaseMotor{

    private CANSparkMax motor;

    public SparkMax(int port){
        motor = new CANSparkMax(port, MotorType.kBrushless);
    }

    @Override
    public void set(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void setInverted(boolean inverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public void setNeutralMode(boolean brake) {
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setPosition(double rotations) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public double getAcceleration() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAcceleration'");
    }

    @Override
    public double getCurrentDraw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrentDraw'");
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
    public void setSmartCurrentLimit(int value) {
        motor.setSmartCurrentLimit(value);
    }

    @Override
    public void enableVoltageCompensation(double value) {
        motor.enableVoltageCompensation(value);
    }

}
