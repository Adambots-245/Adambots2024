/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Generic PhotoEye sensor to hide actual implementation
 */
public class PhotoEye {
    private DigitalInput photoEye;

    public PhotoEye (int port){
        this.photoEye = new DigitalInput(port);
    }

    public boolean isDetecting(){
        return photoEye.get(); 
    }

    public DigitalInput getDigitalInput()
    {
        return photoEye;
    }
}