// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.LEDConstants;
import com.adambots.devices.BaseAddressableLED;
import com.adambots.devices.BaseAddressableLED.AnimationType;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionProcessorSubsystem extends SubsystemBase {
  private BaseAddressableLED ledController;
  private int oldHeartbeat = VisionHelpers.getHeartbeat();
  private int newHeartbeat = VisionHelpers.getHeartbeat();
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
      newHeartbeat = VisionHelpers.getHeartbeat();
      counter = 0;

      // If the old and new heartbeat is same, then the Limelight is not working - change color to indicate this
      if (newHeartbeat == oldHeartbeat) {
        ledController.clearAllAnimations();
        ledController.changeAnimation(AnimationType.Empty);
        ledController.setColor(LEDConstants.purple);
        System.out.println("Limelight possibly frozen");
      } else {
        oldHeartbeat = newHeartbeat;
      }

    } else {
      counter++;
    }
  }
}
