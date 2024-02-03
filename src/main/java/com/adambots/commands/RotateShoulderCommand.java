// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateShoulderCommand extends Command {
  /** Creates a new RotateShoulderCommand. */
  private ArmSubsystem armSubsystem;
  double shoulderIncrement;
  public RotateShoulderCommand(ArmSubsystem armSubsystem, double shoulderIncrement) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    this.shoulderIncrement = shoulderIncrement;
    this.armSubsystem = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.incrementShoulderAngle(shoulderIncrement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
