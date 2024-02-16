

package com.adambots.commands.autonCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.commands.ChangeArmStateCommand;
import com.adambots.commands.RunIntakeCommand;
import com.adambots.commands.SlowIntakeCommand;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeWithAdjustCommand extends SequentialCommandGroup {

  public IntakeWithAdjustCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    super(
      new ChangeArmStateCommand(armSubsystem, ArmConstants.floorState),
      new RunIntakeCommand(intakeSubsystem, armSubsystem,0.3, 0.13)
      // new ParallelDeadlineGroup(new WaitCommand(0.05), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(0.1))),
      // new ParallelDeadlineGroup(new WaitCommand(0.2), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      // new SlowIntakeCommand(intakeSubsystem),
      // new ParallelDeadineGroup(new WaitCommand(0.05), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(0.1))),
      // new ParallelDeadlineGroup(new WaitCommand(0.2), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      // new SlowIntakeCommand(intakeSubsystem)
      // new ParallelDeadlineGroup(new WaitCommand(0.3), new RunCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.1))),
      // new SlowIntakeCommand(intakeSubsystem)
    );                                                                            
  }

}
