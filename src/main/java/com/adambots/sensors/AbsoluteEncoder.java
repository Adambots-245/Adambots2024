/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Generic Absolute Encoder sensor to hide actual implementation and ensure uniform values across subsystems
 */
public class AbsoluteEncoder {
    
    public AbsoluteEncoder (){}

    /**
     * Returns the discrete (does not continue past 360) value of the encoder in degrees
     * @return Discrete value of encoder in degrees
     */
    public double getAbsolutePositionDegrees () {
        System.err.println("WARNING: This method has not been implemented - AbsoluteEncoder.getAbsolutePositionDegrees()");
        return -1;
    }

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    public double getAbsolutePositionRadians () {
        System.err.println("WARNING: This method has not been implemented - AbsoluteEncoder.getAbsolutePositionRadians()");
        return -1;
    }

    /**
     * Returns the discrete (does not continue past 2pi) value of the encoder in radians
     * @return Discrete value of encoder in radians
     */
    public Rotation2d getAbsolutePositionRotation2D () {
        System.err.println("WARNING: This method has not been implemented - AbsoluteEncoder.getAbsolutePositionRotation2D()");
        return null;
    }
}
