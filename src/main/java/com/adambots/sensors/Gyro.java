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
public class Gyro {
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
    public double getContinuousYawDeg () {
        return -gyro.getAngle(); //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in radians
     * <p>
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in radians
     */
    public Rotation2d getContinuousYawRad () {
        return new Rotation2d(Math.toRadians(getContinuousYawDeg()));
    }

    /**
     * Zeros gyroscope yaw
     */
    public void resetYaw () {
        gyro.reset();
    }

    /**
     * Resets the yaw of the gyroscope to the specified value in degrees
     */
    public void resetYawToAngle (double offsetDeg) {
        gyro.setYaw(offsetDeg);
    }

    /**
     * Returns the approximated X, Y and rotation displacement of the robot since last reset
     * @return Approximated Pose2d of the robot
     */
    // public Pose2d getDisplacementPose2D (double offsetDeg) {
    //     return new Pose2d(gyro.getRotation2d(), gyro.getDisplacementY(), getContinuousYawRad());
    // }

    /**
     * Resets the calculated X, Y displacement of the gyro
     */
    // public void resetDisplacement () {
    //     gyro.resetDisplacement();
    // }
}
