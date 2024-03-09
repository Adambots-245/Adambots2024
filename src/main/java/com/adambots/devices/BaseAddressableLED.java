// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.devices;

import edu.wpi.first.wpilibj.util.Color;

/**
 * An interface to an Addressable LED controller such as CANdle or Blinkin.
 */
public interface BaseAddressableLED {

    public enum AnimationType {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        Empty
    }

    void clearAllAnimations();

    void setColor(Color color);
    void setColor(int r, int g, int b);


    void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs);    
    void setLEDs(Color color, int startIdx, int numOfLEDs);


    void configBrightness(double value);

    void changeAnimation(AnimationType toChange);

    void setAnimationSpeed(double speed);

    void update();
}
