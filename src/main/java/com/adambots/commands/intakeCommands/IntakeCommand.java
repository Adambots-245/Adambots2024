// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private double intakeSpeed;
  
  public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
    addRequirements(intakeSubsystem);
    
    this.intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = intakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setMotorSpeed(intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
