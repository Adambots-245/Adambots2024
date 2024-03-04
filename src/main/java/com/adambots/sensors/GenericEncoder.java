/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Generic Absolute Encoder sensor to hide actual implementation and ensure uniform values across subsystems
 */
public class GenericEncoder extends AbsoluteEncoder {
    private DutyCycleEncoder encoder;
    
    public GenericEncoder (int port){
        this.encoder = new DutyCycleEncoder(port); //Defining the encoder using the port passed in
    }

    @Override
    public double getAbsolutePositionDegrees () {
        return encoder.getAbsolutePosition() * 360;
    }

    @Override
    public double getAbsolutePositionRadians () {
        return Units.degreesToRadians(getAbsolutePositionDegrees());
    }

    @Override
    public Rotation2d getAbsolutePositionRotation2D () {
        return new Rotation2d(Units.degreesToRadians(getAbsolutePositionDegrees()));
    }
}
