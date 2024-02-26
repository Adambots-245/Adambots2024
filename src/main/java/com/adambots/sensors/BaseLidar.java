package com.adambots.sensors;

public interface BaseLidar {

    /**
     * Take a measurement and return the distance in cm
     * 
     * @return Distance in cm
     */
    double getDistCm();

    /**
     * Take a measurement and return the distance in inches
     * 
     * @return Distance in inches
     */
    double getDistInches();

}