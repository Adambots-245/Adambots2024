package com.adambots.commands.visionCommands;

import com.adambots.Constants.LEDConstants;
import com.adambots.devices.BaseAddressableLED.AnimationTypes;
import com.adambots.subsystems.LedLightingSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlinkLightsCommand extends SequentialCommandGroup {

  public BlinkLightsCommand(LedLightingSubsystem caNdleSubsystem) {
    super(
        new InstantCommand(() -> caNdleSubsystem.clearAllAnims()),
        new InstantCommand(() -> caNdleSubsystem.changeAnimation(AnimationTypes.SetAll)),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.green)),
        new WaitCommand(0.5),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.off)),
        new WaitCommand(0.5),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.green)),
        new WaitCommand(0.5),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.off)),
        new WaitCommand(0.5),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.green)),
        new WaitCommand(0.5),
        new InstantCommand(() -> caNdleSubsystem.setColor(LEDConstants.off))
    );
  }
}
