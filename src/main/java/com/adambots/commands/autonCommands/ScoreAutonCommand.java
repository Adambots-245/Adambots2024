// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.commands.ChangeArmStateCommand;
import com.adambots.commands.RotateShoulderCommand;
import com.adambots.commands.RunIntakeCommand;
import com.adambots.commands.SlowIntakeCommand;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.Constants.ArmConstants.State;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAutonCommand extends SequentialCommandGroup {
  
  /** Creates a new ScoreAutonSpeakerCommand. */
  public ScoreAutonCommand(ArmSubsystem armSubsystem, State armState) {
    super(
      new ParallelDeadlineGroup(new WaitCommand(1.5), new RotateShoulderCommand(armSubsystem, 1)),
      new ChangeArmStateCommand(armSubsystem, armState)
    );   

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
