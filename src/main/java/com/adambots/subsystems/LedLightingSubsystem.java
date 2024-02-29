// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.devices.BaseAddressableLED;
import com.adambots.devices.BaseAddressableLED.AnimationType;
import com.adambots.devices.BaseAddressableLED.AnimationTypes;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedLightingSubsystem extends SubsystemBase {
private BaseAddressableLED ledController;

  @Override
  public void periodic() {
    if (ledController == null) {
      return;
    }

    ledController.update();
  }


  public LedLightingSubsystem(BaseAddressableLED ledController) {
    this.ledController = ledController;

    // setColor(LEDConstants.adambotsYellow); //Adambots Yellow
  }

  public void setColor(Color color) {
    ledController.setColor(color);
  }

  /**
   * Set the colors for the LED Strip
   * 
   * @param r - Red (0 to 255)
   * @param g - Green (0 to 255)
   * @param b - Blue (0 to 255)
   */
  public void setColor(int r, int g, int b) {
    ledController.setColor(r, g, b);
  }

  /**
   * Set an individual LED to a specified color
   * 
   * @param r
   * @param g
   * @param b
   * @param startIdx  - 0 based index for starting LED. The strip starts at 8 for CANdle.
   * @param numOfLEDs - Number of LEDs to light up with this color
   */
  public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs) {

    ledController.setLEDs(r, g, b, startIdx, numOfLEDs);
  }

  public void setStrobe(Color color) {
    ledController.setColor(color);
    ledController.changeAnimation(AnimationTypes.Strobe);
  }

  // public void setmodulateVBatOutput(double dutyCycle) {
  // this.vbatOutput = dutyCycle;
  // }

  /* Wrappers so we can access the CANdle from the subsystem */
  // public double getVbat() {
  // return candle.getBusVoltage();
  // }

  // public double get5V() {
  // return candle.get5VRailVoltage();
  // }

  // public double getCurrent() {
  // return candle.getCurrent();
  // }

  // public double getTemperature() {
  // return candle.getTemperature();
  // }

  public void configBrightness(double value) {
    ledController.configBrightness(value);
  }

  // public void configLos(boolean disableWhenLos) {
  // candle.configLOSBehavior(disableWhenLos, 0);
  // }

  // public void configLedType(LEDStripType type) {
  // candle.configLEDType(type, 0);
  // }

  // public void configStatusLedBehavior(boolean offWhenActive) {
  // candle.configStatusLedState(offWhenActive, 0);
  // }

  public void changeAnimation(AnimationType toChange) {
    ledController.changeAnimation(toChange);
  }

  public void clearAllAnims() {
    ledController.clearAllAnimations();
  }

  /**
   * Set the speed of animation - 0.0 to 1.0
   * 
   * @param speed
   */
  public void setAnimationSpeed(double speed) {
    ledController.setAnimationSpeed(speed);
  }
}