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
    private Boolean inverted;

    public PhotoEye (int port, boolean inverted){
        this.photoEye = new DigitalInput(port);
        this.inverted = inverted;
    }

    public boolean isDetecting(){
        if (inverted) {
            return photoEye.get(); 
        }
        return photoEye.get(); 
    }

    public DigitalInput getDigitalInput()
    {
        return photoEye;
    }
}