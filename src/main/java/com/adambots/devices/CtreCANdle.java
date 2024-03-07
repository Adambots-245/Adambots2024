// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.devices;

import com.adambots.Constants.LEDConstants;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A wrapper class for CANdle LED controller from CTRE. Currently requires
 * Phoenix 5.json vendordep
 */
public class CtreCANdle implements BaseAddressableLED {
    private CANdle candle;
    private int candleChannel;
    private Animation toAnimate = null;
    private double animateSpeed;
    private boolean setAnim = false;
    private double vbatOutput = 0.0;

    private int red;
    private int green;
    private int blue;

    private static final int LEDS_IN_STRIP = LEDConstants.LEDS_IN_STRIP;

    public enum CANdleAnimationType {
        ColorFlow(0, new ColorFlowAnimation(255, 150, 0, 0, 1, LEDS_IN_STRIP, Direction.Forward, 8)),
        Fire(1, new FireAnimation(0.5, 0.7, LEDS_IN_STRIP, 0.8, 0.5, true, 8)),
        Larson(2, new LarsonAnimation(255, 150, 0, 100, 0.001, LEDS_IN_STRIP, BounceMode.Front, 30, 0)),
        Rainbow(3, new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, true, 8)),
        RgbFade(4, new RgbFadeAnimation(0.7, 0.4, LEDS_IN_STRIP, 8)),
        SingleFade(5, new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_IN_STRIP, 8)),
        Strobe(6, new StrobeAnimation(0, 0, 255, 0, 0, LEDS_IN_STRIP, 8)),
        Twinkle(7, new TwinkleAnimation(0, 255, 0, 0, 1, LEDS_IN_STRIP, TwinklePercent.Percent100, 0)),
        TwinkleOff(8, new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_IN_STRIP, TwinkleOffPercent.Percent76, 8)),
        Empty(-1, null);

        public final Animation animation;
        public final int channel;

        CANdleAnimationType(int channel, Animation animation) {
            this.animation = animation;
            this.channel = channel;
        }
    };

    public CtreCANdle(int canPort) {
        candle = new CANdle(canPort);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.GRB; // the BTF-Lighting LED strip uses GRB format
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;

        candle.configAllSettings(configAll, 100);

        clearAllAnimations();
        // changeAnimation(CANdleAnimationTypes.SetAll);
        configBrightness(1);
    }

    public void clearAnimation(int slot){
        candle.clearAnimation(slot);
    }

    public void clearAllAnimations() {
        for (int i = 0; i < 10; ++i) {
            clearAnimation(i);
        }

        this.candleChannel = -1;
    }

    public void setColor(Color color) {
        setColor(getColorValue(color.red), getColorValue(color.green), getColorValue(color.blue));

        setAnim = false;
    }

    /**
     * Set the colors for the LED Strip
     * 
     * @param r - Red (0 to 255)
     * @param g - Green (0 to 255)
     * @param b - Blue (0 to 255)
     */
    public void setColor(int r, int g, int b) {
        this.red = MathUtil.clamp(r, 0, 255);
        this.green = MathUtil.clamp(g, 0, 255);
        this.blue = MathUtil.clamp(b, 0, 255);

        setAnim = false;
    }

    public void setStrobe(Color color) {
        candleChannel = 6;
        toAnimate = new StrobeAnimation(getColorValue(color.red), getColorValue(color.green), getColorValue(color.blue), 0, 0, LEDS_IN_STRIP, 8);
    }

    /**
     * Set an individual LED to a specified color
     * 
     * @param r
     * @param g
     * @param b
     * @param startIdx  - 0 based index for starting LED. The strip starts at 8.
     * @param numOfLEDs - Number of LEDs to light up with this color
     */
    public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs) {
        r = MathUtil.clamp(r, 0, 255);
        g = MathUtil.clamp(g, 0, 255);
        b = MathUtil.clamp(b, 0, 255);

        if (startIdx < 0 || startIdx > LEDS_IN_STRIP)
            startIdx = 0;

        if (numOfLEDs < 0 || numOfLEDs > LEDS_IN_STRIP)
            numOfLEDs = LEDS_IN_STRIP;

        candle.setLEDs(r, g, b, 0, startIdx, numOfLEDs);
    }

    public void setLEDs(Color color, int startIdx, int numOfLEDs){
        setLEDs(getColorValue(color.red), getColorValue(color.green), getColorValue(color.blue), startIdx, numOfLEDs);
    }

    /**
     * Return integer value from 0 to 255 from a 0 to 1 double value system that Color object uses
     * @param colorVal
     * @return
     */
    private int getColorValue(double colorVal){
        return (int)colorVal * 255;
    }

    public void configBrightness(double value) {
        candle.configBrightnessScalar(value, 0);
    }

    public void changeAnimation(AnimationType toChange) {
        CANdleAnimationType animationType = CANdleAnimationType.valueOf(toChange.toString());
        toAnimate = animationType.animation;
        candleChannel = animationType.channel;
    }

    /**
     * Set the speed of animation - 0.0 to 1.0
     * 
     * @param speed
     */
    public void setAnimationSpeed(double speed) {
        animateSpeed = speed;
    }

    /**
     * Update the LED animations and color change. Call this from Subsystem's
     * periodic.
     */
    public void update() {
        if (candle == null) {
            return;
        }

        if (toAnimate == null) {
            if (!setAnim) {
                /* Only setLEDs once, because every set will transmit a frame */
                setAnim = true;
                candle.setLEDs((int) (red), (int) (green), (int) (blue), 0, 0, LEDS_IN_STRIP);
            }
        } else {
            toAnimate.setSpeed((animateSpeed + 1) * 0.5);
            candle.animate(toAnimate, candleChannel);
            setAnim = false;
        }

        candle.modulateVBatOutput(vbatOutput);
    }
}
