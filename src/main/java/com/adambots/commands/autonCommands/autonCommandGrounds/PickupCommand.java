// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
