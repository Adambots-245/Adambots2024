

package com.adambots.commands.autonCommands;

import com.adambots.commands.SlowIntakeCommand;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AdjustNoteCommand extends SequentialCommandGroup {

  public AdjustNoteCommand(IntakeSubsystem intakeSubsystem) {
    super(
      new SlowIntakeCommand(intakeSubsystem),
      new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1)),
      new WaitCommand(0.25),
      new SlowIntakeCommand(intakeSubsystem),
      new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1)),
      new WaitCommand(0.25),
      new SlowIntakeCommand(intakeSubsystem),
      new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1)),
      new WaitCommand(0.25),
      new SlowIntakeCommand(intakeSubsystem)
    );                                                                            
  }

}
