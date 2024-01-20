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

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Generic gyro sensor to hide actual implementation and ensure uniform values across subsystems
 */
public class Gyro {
    private AHRS gyro;
    
    public Gyro (){
        this.gyro = new AHRS(Port.kMXP); //Defining the gyroscrope using the MXP port on the roborio (mounted directly to top of Rio)
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in radians
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in radians
     */
    public Rotation2d getContinuousYawRad () {
        //References the deg function to ensure no discrepency between the measurments
        return new Rotation2d(Math.toRadians(getContinuousYawDeg()));
    }

    /**
     * Returns the continuous (Continues from 360-361) value of the gyroscope in degrees
     * Ensure CCW is a positive value change
     * @return Continous value of gyroscope in degrees
     */
    public double getContinuousYawDeg () {
        return -gyro.getAngle(); //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
    }

    /**
     * Resets the yaw gyroscope
     */
    public void resetYaw () {
        // gyro.reset();
        gyro.zeroYaw();
        // gyro.setAngleAdjustment(0.0);
    }

    /**
     * Resets the yaw of the gyroscope to the user specified value in degrees
     */
    public void resetYawToAngle (double offsetDeg) {
        gyro.reset();
        gyro.setAngleAdjustment(offsetDeg);
    }

    /**
     * Returns the approximated X, Y and rotation displacement of the robot since last reset
     * @return Approximated Pose2d of the robot
     */
    public Pose2d getDisplacementPose2D (double offsetDeg) {
        return new Pose2d(gyro.getDisplacementX(), gyro.getDisplacementY(), getContinuousYawRad());
    }

    /**
     * Resets the calculated X, Y displacement of the gyro
     */
    public void resetDisplacement () {
        gyro.resetDisplacement();
    }
    
    /**
     * Calibrates the gyroscope
     * Should only be performed on initialization - may take up to 2 seconds to complete
     */
    public void calibrate () {
        gyro.calibrate();
    }
}
