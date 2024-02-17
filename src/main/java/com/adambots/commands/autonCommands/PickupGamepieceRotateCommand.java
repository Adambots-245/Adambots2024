package com.adambots.commands.autonCommands;

import com.adambots.Constants.VisionConstants;
import com.adambots.commands.visionCommands.AlignNoteVertCommand;
import com.adambots.commands.visionCommands.AlignRotateCommand;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupGamepieceRotateCommand extends SequentialCommandGroup {

  public PickupGamepieceRotateCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
        new AlignRotateCommand(drivetrainSubsystem, false, true, VisionConstants.noteLimelite),
        new AlignNoteVertCommand(drivetrainSubsystem)
    );
  }
}
