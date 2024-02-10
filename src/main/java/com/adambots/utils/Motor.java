// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public abstract class Motor {

    public Motor(){
        
    }

    public void set(double speed){System.out.println("WARNING: Functionality for this method has not been implemented.");}

    public void setInverted(boolean inverted){System.out.println("WARNING: Functionality for this method has not been implemented.");}

    public void setNeutralMode(boolean brake){System.out.println("WARNING: Functionality for this method has not been implemented.");}

    public double getPosition(){System.out.println("WARNING: Functionality for this method has not been implemented.");return 0;}

    public double getVelocity(){System.out.println("WARNING: Functionality for this method has not been implemented.");return 0;}

    public double getAcceleration(){System.out.println("WARNING: Functionality for this method has not been implemented.");return 0;}

}
