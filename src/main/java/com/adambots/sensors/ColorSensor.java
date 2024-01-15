/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;

import com.adambots.RobotMap;
import com.revrobotics.ColorMatch;

/**
 * Generic color sensor to hide actual implementation
 */
public class ColorSensor extends BaseSensor {

    private final static ColorSensorV3 colorSensor = new ColorSensorV3(RobotMap.I2C_PORT);

    private final static ColorMatch colorMatcher = new ColorMatch();

    public ColorSensor(){
    
        //Init Color Matcher code
        colorMatcher.addColorMatch(Color.kBlue);
        // colorMatcher.addColorMatch(Constants.GREEN_TARGET);
        colorMatcher.addColorMatch(Color.kRed);
        // colorMatcher.addColorMatch(Constants.YELLOW_TARGET);

    }

	public Color getColor() {
		return colorSensor.getColor();
	}

	public Color matchClosestColor(Color detectedColor) {
        
		return colorMatcher.matchClosestColor(detectedColor).color;
    }

	public double getProximity() {
		return colorSensor.getProximity();
	}
    
}
