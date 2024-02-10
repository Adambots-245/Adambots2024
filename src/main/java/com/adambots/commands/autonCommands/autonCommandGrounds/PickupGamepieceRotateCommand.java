package com.adambots.commands.autonCommands.autonCommandGrounds;

import com.adambots.commands.autonCommands.AlignRotateCommand;
import com.adambots.Constants.VisionConstants;
import com.adambots.commands.autonCommands.AlignNoteVertCommand;
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
