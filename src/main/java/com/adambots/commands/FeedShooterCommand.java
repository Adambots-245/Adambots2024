// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterCommand extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private int inc = 0;
  
  public FeedShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(intakeSubsystem, shooterSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getShooterVelocity() >= 60) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.3);
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setGroundIntakeMotorSpeed(0);
    intakeSubsystem.setNote(false);
    new RunShooterCommand(intakeSubsystem, shooterSubsystem, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 20;
  }
}
