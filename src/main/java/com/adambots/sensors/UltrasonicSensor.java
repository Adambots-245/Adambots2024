// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.sensors;

import com.adambots.utils.Dash;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class UltrasonicSensor {
    // The handle to access the sensor
    private final AnalogInput rangefinder;
    
    // The scaling factor:  distance in inches = volts returned / SCALING_FACTOR
    // 5120 mm per 5V or 512 cm per 5V. The 24/23 is a correction factor to adjust the reading
    private final int SCALING_FACTOR = 512/5*24/23;
    // private final double SCALING_FACTOR = 5 / RobotController.getVoltage5V();
    
    /** Creates a new ultrasonic sensor hooked up to <code>portNumber</code> on the analog breakout.
     * @params portNumber The port number on the breakout.
     */
    public UltrasonicSensor(int portNumber){
        rangefinder = new AnalogInput(portNumber);
        rangefinder.setOversampleBits(2); // Completely arbitrary
        rangefinder.setAverageBits(5); // Ditto
        Dash.add("Scaling Factor", () ->  SCALING_FACTOR);
        Dash.add("AVolts", () -> rangefinder.getAverageVoltage());
        Dash.add("Volts", () -> rangefinder.getVoltage());
        Dash.add("I", () -> rangefinder.getAverageVoltage() * SCALING_FACTOR);
    }
    
    /** Returns the distance measured in inches.  */
    public double getInches(){
        // double volts = rangefinder.getAverageVoltage();
        return getCentimeters() / 2.54;
    }
    
    /** Returns the distance measured in inches.  */
    public double getCentimeters(){
        double volts = rangefinder.getAverageVoltage();
        return (double) (volts * SCALING_FACTOR);
    }
    
    /** Returns the distance measured in feet.  */
    public double getFeet(){
        return this.getInches() / 12.0;
    }
    
    public String toString(){
        // return (int)getFeet() + "' " + (int)(getInches() - (int) getFeet()*12) + "\"";
        return String.format("%d' %d\n", getFeet(), getInches());
    }
}
