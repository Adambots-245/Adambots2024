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

    /**
     * Define a PhotoEye sensor on a given DigitalInput port. 
     * If you set up the PhotoEye to return true when not detecting as to ensure a non-critical failure mode, 
     * you can define it as inverted to maintain the expected (reads true when detecting) return in code.
     * <p>
     * (photoeyes return true when disconnected)
     * @param port
     * The digital input port of the PhotoEye
     * @param inverted
     * Whether or not to invert the PhotoEye return
     */
    public PhotoEye (int port, boolean inverted){
        this.photoEye = new DigitalInput(port);
        this.inverted = inverted;
    }

    /**
     * Returns the detection of the photoeye with regards to whether or not the PhotoEye is defined as inverted
     *
     * @return Whether or not the PhotoEye is detecting something
     */
    public boolean isDetecting(){
        if (inverted) return !photoEye.get();
        return photoEye.get(); 
    }
}