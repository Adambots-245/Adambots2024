// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.devices;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.util.Color;

/**
 * An interface to an Addressable LED controller such as CANdle or Blinkin.
 */
public interface BaseAddressableLED {

    public interface AnimationType {
        // Color color = Color.kWhite;
        // int speed = 0;
        // int numOfLEDs = 0;
        // int ledOffset = 0;
    };

    public interface AnimationTypes extends AnimationType {
        AnimationType ColorFlow = null;
        AnimationType Fire = null;
        AnimationType Larson = null;
        AnimationType Rainbow = null;
        AnimationType RgbFade = null;
        AnimationType SingleFade = null;
        AnimationType Strobe = null;
        AnimationType Twinkle = null;
        AnimationType TwinkleOff = null;
        AnimationType SetAll = null;
        AnimationType Empty = null;

        AnimationType animation = null;
    };

    public Map<String, Object> animationTypes = new HashMap<String, Object>();

    void clearAnimation(int index);

    void clearAllAnimations();

    void setColor(Color color);

    void setColor(int r, int g, int b);

    void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs);

    void configBrightness(double value);

    void changeAnimation(AnimationType toChange);

    void setAnimationSpeed(double speed);

    void update();
}
