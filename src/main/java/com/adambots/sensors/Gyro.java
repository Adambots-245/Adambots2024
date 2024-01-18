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
   * Returns the continuous (does not loop around past pi) value of the gyroscope in radians
   * Ensure CCW is a positive value change
   * @return Continous value of gyroscope in radians
   */
  public Rotation2d getGyroYaw () {
    return new Rotation2d(Math.toRadians(-gyro.getAngle()+180)); //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
  }
    public double getGyroYawDeg () {
    return (-gyro.getAngle()+180); //COUNTERCLOCKWISE NEEDS TO BE POSITIVE
  }
  public void reset () {
    gyro.reset();
  }

    // public static void main(String[] args) {
    //     PhotoEye pe1 = new PhotoEye(6);
    //     PhotoEye pe2 = new PhotoEye(7);

    //     while (true){
    //         System.out.println("PhotoEye1 Detecting: " + pe1.isDetecting());
    //         System.out.println("PhotoEye2 Detecting: " + pe2.isDetecting());
    //     }
    // }
}