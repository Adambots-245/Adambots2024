/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class Gyro extends BaseSensor {
    private AHRS gyro;
    
    public Gyro (){
        this.gyro = new AHRS(Port.kMXP);

    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in radians
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in radians
     */
    public Rotation2d getContinuousYawRad () {
        return new Rotation2d(Math.toRadians(getContinuousYawDeg()));
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in degrees
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in degrees
     */
    public double getContinuousYawDeg () {
        return -gyro.getAngle()+180; //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
    }

    /**
     * Resets the yaw of the gyroscope to 0
     */
    public void reset () {
        gyro.reset();
    }

    /**
     * Resets the yaw of the gyroscope to the user specified value in degrees
     */
    public void resetToAngle (double offsetDeg) {
        gyro.reset();
        gyro.setAngleAdjustment​(offset)
    }

    /**
     * Calibrates the gyroscope
     * Should only be performed on initialization - may take up to 2 seconds to complete
     */
    public void calibrate () {
        gyro.calibrate();
    }
}
