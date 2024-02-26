package com.adambots.sensors;

public interface BaseUltrasonicSensor {

    /** Returns the distance measured in inches.  */
    double getInches();

    /** Returns the distance measured in inches.  */
    double getCentimeters();

    /** Returns the distance measured in feet.  */
    double getFeet();

}