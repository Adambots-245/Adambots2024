// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.LEDConstants;
import com.adambots.devices.BaseAddressableLED;
import com.adambots.devices.BaseAddressableLED.AnimationTypes;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionProcessorSubsystem extends SubsystemBase {
  private BaseAddressableLED ledController;
  private int oldHeartbeat = VisionHelpers.getHeatbeat();
  private int newHeartbeat = VisionHelpers.getHeatbeat();
  private int counter = 0;
  private final int COUNTER_SIZE = 5;

  /** Creates a new VisionProcessorSubsystem. */
  public VisionProcessorSubsystem(BaseAddressableLED ledController) {
    this.ledController = ledController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (counter > COUNTER_SIZE) {
      newHeartbeat = VisionHelpers.getHeatbeat();
      counter = 0;

      // If the old and new heartbeat is same, then the Limelight is not working - change color to indicate this
      if (newHeartbeat == oldHeartbeat) {
        ledController.clearAllAnimations();
        ledController.changeAnimation(AnimationTypes.SetAll);
        ledController.setColor(LEDConstants.purple);
      } else {
        oldHeartbeat = newHeartbeat;
      }

    } else {
      counter++;
    }
  }
}
