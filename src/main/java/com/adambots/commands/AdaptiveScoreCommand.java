// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AdaptiveScoreCommand extends Command {
  /** Creates a new AdaptiveScoreCommand. */
  ShooterSubsystem shooterSubsystem;
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;

  private Command ampScoreCommand;
  

  public AdaptiveScoreCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(armSubsystem, shooterSubsystem, intakeSubsystem);

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (armSubsystem.getCurrentStateName() == "speaker") {
      new FeedShooterCommand(intakeSubsystem, shooterSubsystem).schedule();
    } else if (armSubsystem.getCurrentStateName() == "amp") {
      ampScoreCommand = new AmpScoreCommand(intakeSubsystem, shooterSubsystem);
      ampScoreCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (ampScoreCommand.isScheduled()) {
      ampScoreCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
