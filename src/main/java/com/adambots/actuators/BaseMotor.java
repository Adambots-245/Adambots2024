// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

/** Add your docs here. */
public abstract class BaseMotor {

    public BaseMotor(){}

    public void set(double speed){ 
        System.err.println("WARNING: This method has not been implemented - BaseMotor.set()");
    }

    public void setInverted(boolean inverted){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.setInverted()");
    }

    public void setBrakeMode(boolean brake){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.setBrakeMode()");
    }

    public void setPosition(double rotations){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.setPosition()");
    }

    public double getPosition(){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.getPosition()");
        return -1;
    }

    public double getVelocity(){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.getVelocity()");
        return -1;
    }

    public double getCurrentDraw(){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.getCurrentDraw()");
        return -1;
    }

    public boolean getForwardLimitSwitch() {
        System.err.println("WARNING: This method has not been implemented - BaseMotor.getForwardLimitSwitch()");
        return false;
    }
    
    public boolean getReverseLimitSwitch(){
        System.err.println("WARNING: This method has not been implemented - BaseMotor.getReverseLimitSwitch()");
        return false;
    }

}
