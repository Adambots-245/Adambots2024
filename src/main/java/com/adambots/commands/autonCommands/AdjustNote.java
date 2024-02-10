

package com.adambots.commands.autonCommands;

import com.adambots.commands.RunIntakeCommand;
import com.adambots.commands.SlowIntakeCommand;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AdjustNote extends SequentialCommandGroup {

  public AdjustNote(IntakeSubsystem intakeSubsystem) {
    super(
      new RunIntakeCommand(intakeSubsystem,0.2),
      // new ParallelDeadlineGroup(new WaitCommand(0.05), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(0.1))),
      new ParallelDeadlineGroup(new WaitCommand(0.2), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      new SlowIntakeCommand(intakeSubsystem, 0.09),
      // new ParallelDeadlineGroup(new WaitCommand(0.05), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(0.1))),
      new ParallelDeadlineGroup(new WaitCommand(0.2), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      new SlowIntakeCommand(intakeSubsystem, 0.09)
      // new ParallelDeadlineGroup(new WaitCommand(0.3), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      // new SlowIntakeCommand(intakeSubsystem)
    );                                                                            
  }

}
