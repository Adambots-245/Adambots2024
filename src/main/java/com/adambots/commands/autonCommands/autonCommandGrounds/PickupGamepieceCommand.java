package com.adambots.commands.autonCommands.autonCommandGrounds;

import com.adambots.commands.autonCommands.AlignNoteHorizCommand;
import com.adambots.commands.autonCommands.AlignNoteVertCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupGamepieceCommand extends SequentialCommandGroup {

  public PickupGamepieceCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
      new AlignNoteHorizCommand(drivetrainSubsystem),
      new AlignNoteVertCommand(drivetrainSubsystem)
    );
  }
}
