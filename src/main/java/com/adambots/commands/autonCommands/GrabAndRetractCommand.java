// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.commands.ChangeArmStateCommand;
import com.adambots.commands.RotateShoulderCommand;
import com.adambots.commands.RotateWristCommand;
import com.adambots.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAndRetractCommand extends SequentialCommandGroup {
  /** Creates a new GrabAndRetractCommand. */
  public GrabAndRetractCommand(ArmSubsystem armSubsystem, State armState) {
    super(
      // new ParallelRaceGroup(new WaitCommand(1), new RotateWristCommand(armSubsystem, 10, true), new RotateShoulderCommand(armSubsystem, 1, true)),
      new ChangeArmStateCommand(armSubsystem, ArmConstants.liftNoteState),
      new WaitCommand(0.5),
      new ChangeArmStateCommand(armSubsystem, armState)
    );   
    
  }
}
