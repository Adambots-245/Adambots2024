// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;


/** Add your docs here. */
public abstract class BaseMotor {

    public BaseMotor(){}

    public void set(double speed){System.err.println("set WARNING: Functionality for this method has not been implemented.");}

    public void setInverted(boolean inverted){System.err.println("inverted WARNING: Functionality for this method has not been implemented.");}

    public void setNeutralMode(boolean brake){System.err.println("neutral mode WARNING: Functionality for this method has not been implemented.");}

    public void setPosition(double rotations){System.err.println(" set pos WARNING: Functionality for this method has not been implemented.");}

    public double getPosition(){System.err.println("get pos WARNING: Functionality for this method has not been implemented."); return 0;}

    public double getVelocity(){System.err.println("get vel WARNING: Functionality for this method has not been implemented."); return 0;}

    public double getAcceleration(){System.err.println("get acclWARNING: Functionality for this method has not been implemented."); return 0;}

    public double getCurrentDraw(){System.err.println(" get currentWARNING: Functionality for this method has not been implemented."); return 0;}

    public boolean getForwardLimitSwitch(){System.err.println("get lim f WARNING: Functionality for this method has not been implemented."); return false;}
    
    public boolean getReverseLimitSwitch(){System.err.println("get lim r WARNING: Functionality for this method has not been implemented."); return false;}

}
