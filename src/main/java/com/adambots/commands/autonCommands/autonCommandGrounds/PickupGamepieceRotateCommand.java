package com.adambots.commands.autonCommands.autonCommandGrounds;

import com.adambots.commands.autonCommands.AlignNoteRotateCommand;
import com.adambots.commands.autonCommands.AlignNoteVertCommand;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupGamepieceRotateCommand extends SequentialCommandGroup {

  public PickupGamepieceRotateCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
        new AlignNoteRotateCommand(drivetrainSubsystem, false, true),
        new AlignNoteVertCommand(drivetrainSubsystem)
      );
  }
}
