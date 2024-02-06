

package com.adambots.commands.autonCommands;

import com.adambots.commands.RunIntakeCommand;
import com.adambots.commands.SimpleIntakeCommand;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AdjustNote extends SequentialCommandGroup {

  public AdjustNote(IntakeSubsystem intakeSubsystem) {
    super(
      new RunIntakeCommand(intakeSubsystem, 0.3),
      new RunIntakeCommand(intakeSubsystem, -0.1),
      new ParallelDeadlineGroup(new WaitCommand(0.22), new SimpleIntakeCommand(intakeSubsystem, 0.1))
    );
  }
}
