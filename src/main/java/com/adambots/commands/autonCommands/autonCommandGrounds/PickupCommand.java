package com.adambots.commands.autonCommands.autonCommandGrounds;

import com.adambots.commands.autonCommands.AlignNoteCommand;
import com.adambots.commands.autonCommands.AlignNoteDistanceCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickupCommand extends SequentialCommandGroup {

  public PickupCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
      new AlignNoteCommand(drivetrainSubsystem),
      new AlignNoteDistanceCommand(drivetrainSubsystem)
    );
  }
}
