package com.adambots.commands.autonCommands.autonCommandGroups;

import com.adambots.commands.visionCommands.AlignNoteHorizCommand;
import com.adambots.commands.visionCommands.AlignNoteVertCommand;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupGamepieceStrafeCommand extends SequentialCommandGroup {

  public PickupGamepieceStrafeCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
      new AlignNoteHorizCommand(drivetrainSubsystem),
      new AlignNoteVertCommand(drivetrainSubsystem)
    );
  }
}
