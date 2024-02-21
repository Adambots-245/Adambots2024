// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;


/** Add your docs here. */
public abstract class BaseMotor {

    public BaseMotor(){}

    public void set(double speed){System.err.println("WARNING: Functionality for this method has not been implemented.");}

    public void setInverted(boolean inverted){System.err.println("WARNING: Functionality for this method has not been implemented.");}

    public void setNeutralMode(boolean brake){System.err.println("WARNING: Functionality for this method has not been implemented.");}

    public void setPosition(double rotations){System.err.println("WARNING: Functionality for this method has not been implemented.");}

    public double getPosition(){System.err.println("WARNING: Functionality for this method has not been implemented."); return 0;}

    public double getVelocity(){System.err.println("WARNING: Functionality for this method has not been implemented."); return 0;}

    public double getAcceleration(){System.err.println("WARNING: Functionality for this method has not been implemented."); return 0;}

    public double getCurrentDraw(){System.err.println("WARNING: Functionality for this method has not been implemented."); return 0;}

    public Boolean getForwardLimit(){System.err.println("WARNING: Functionality for this method has not been implemented."); return false;}
    
    public Boolean getReverseLimit(){System.err.println("WARNING: Functionality for this method has not been implemented."); return false;}

}
