/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Generic gyro sensor to hide actual implementation and ensure uniform values across subsystems
 */
public class Gyro implements BaseGyro {
    private Pigeon2 gyro;
    
    public Gyro (int CANport){
        this.gyro = new Pigeon2(CANport); //Defining the gyroscrope using the configured CAN ID
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in degrees
     * <p>
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in degrees
     */
    @Override
    public double getContinuousYawDeg () {
        return -gyro.getAngle(); //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in radians
     * <p>
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in radians
     */
    @Override
    public Rotation2d getContinuousYawRad () {
        return new Rotation2d(Math.toRadians(getContinuousYawDeg()));
    }

    /**
     * Zeros gyroscope yaw
     */
    @Override
    public void resetYaw () {
        gyro.reset();
    }

    /**
     * Resets the yaw of the gyroscope to the specified value in degrees
     */
    @Override
    public void resetYawToAngle (double offsetDeg) {
        gyro.setYaw(offsetDeg);
    }

    /**
     * Returns the measured roll value of the gyroscope in degrees
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation
     * @return Roll value of gyroscope in degrees
     */
    @Override
    public double getPitch () {
        return gyro.getRoll().getValueAsDouble();
        // Pitch and roll values are switched for pigeon
    }

    /**
     * Returns the measured pitch value of the gyroscope in degrees
     * <p>
     * Keep in mind roll and pitch will change depending on robot rotation
     * @return Pitch value of gyroscope in degrees
     */
    @Override
    public double getRoll () {
        return gyro.getPitch().getValueAsDouble();
        // Pitch and roll values are switched for pigeon

    }
}
