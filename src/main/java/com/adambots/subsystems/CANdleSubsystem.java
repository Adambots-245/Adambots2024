// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private static final int LEDS_IN_STRIP = LEDConstants.LEDS_IN_STRIP;
  private CANdle candle;
  private Animation toAnimate = null;
  
  @Override
  public void periodic() {    
    if (candle == null) {
      return;
    }

    if (toAnimate == null) {   
      if(!setAnim) {
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

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  private int red = 0;
  private int green = 0;
  private int blue = 0;
  private double vbatOutput = 0.0;
  private int candleChannel;
  private boolean animDirection = true;
  private boolean setAnim = false;
  private double animateSpeed = 0.1;

  public CANdleSubsystem(CANdle candleDevice) {
    this.candle = candleDevice;

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = true;
    configAll.stripType = LEDStripType.GRB; // the BTF-Lighting LED strip uses GRB format
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;

    candleDevice.configAllSettings(configAll, 100);

    clearAllAnims();
    changeAnimation(AnimationTypes.SetAll);
    setColor(LEDConstants.adambotsYellow); //Adambots Yellow
  }

  public void setColor(Color color) {
    setColor((int) color.red, (int) color.green, (int) color.blue);

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

  /**
   * Set an individual LED to a specified color
   * @param r
   * @param g
   * @param b
   * @param startIdx - 0 based index for starting LED. The strip starts at 8.
   * @param numOfLEDs - Number of LEDs to light up with this color
   */
  public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs){
    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);
    
    if (startIdx < 0 || startIdx > LEDS_IN_STRIP)
      startIdx = 0;
    
    if (numOfLEDs < 0 || numOfLEDs > LEDS_IN_STRIP)
      numOfLEDs = LEDS_IN_STRIP;

    candle.setLEDs(r, g, b, 0, startIdx, numOfLEDs);
  }

  // public void setmodulateVBatOutput(double dutyCycle) {
  //   this.vbatOutput = dutyCycle;
  // }

  /* Wrappers so we can access the CANdle from the subsystem */
  // public double getVbat() {
  //   return candle.getBusVoltage();
  // }

  // public double get5V() {
  //   return candle.get5VRailVoltage();
  // }

  // public double getCurrent() {
  //   return candle.getCurrent();
  // }

  // public double getTemperature() {
  //   return candle.getTemperature();
  // }

  // public void configBrightness(double percent) {
  //   candle.configBrightnessScalar(percent, 0);
  // }

  // public void configLos(boolean disableWhenLos) {
  //   candle.configLOSBehavior(disableWhenLos, 0);
  // }

  // public void configLedType(LEDStripType type) {
  //   candle.configLEDType(type, 0);
  // }

  // public void configStatusLedBehavior(boolean offWhenActive) {
  //   candle.configStatusLedState(offWhenActive, 0);
  // }

  public void changeAnimation(AnimationTypes toChange) {
    switch (toChange) {
      default:
      case ColorFlow:
        candleChannel = 0;
        toAnimate = new ColorFlowAnimation(255, 150, 0, 0, 0.7, LEDS_IN_STRIP, Direction.Forward, 8);
        break;
      case Fire:
        candleChannel = 1;
        toAnimate = new FireAnimation(0.5, 0.7, LEDS_IN_STRIP, 0.8, 0.5, animDirection, 8);
        break;
      case Larson:
        candleChannel = 2;
        toAnimate = new LarsonAnimation(255, 150, 0, 100, 0.001, LEDS_IN_STRIP, BounceMode.Front, 30, 8);
        break;
      case Rainbow:
        candleChannel = 3;
        toAnimate = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, animDirection, 8);
        break;
      case RgbFade:
        candleChannel = 4;
        toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_IN_STRIP, 8);
        break;
      case SingleFade:
        candleChannel = 5;
        toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_IN_STRIP, 8);
        break;
      case Strobe:
        candleChannel = 6;
        toAnimate = new StrobeAnimation(255, 255, 255, 1, 1, LEDS_IN_STRIP, 8);
        break;
      case Twinkle:
        candleChannel = 7;
        toAnimate = new TwinkleAnimation(0, 255, 0, 0, 0.4, LEDS_IN_STRIP, TwinklePercent.Percent100, 0);
        break;
      case TwinkleOff:
        candleChannel = 8;
        toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_IN_STRIP, TwinkleOffPercent.Percent76, 8);
        break;
      case Empty:
        candleChannel = 9;
        toAnimate = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, animDirection, 8);
        break;
      case SetAll:
        toAnimate = null;
        break;
    }
  }

  public void clearAllAnims() {
    for(int i = 0; i < 10; ++i) {
      candle.clearAnimation(i);
    }
  }

  /**
   * Set the speed of animation - 0.0 to 1.0
   * @param speed
   */
  public void setAnimationSpeed(double speed){
    animateSpeed = speed;
  }
}