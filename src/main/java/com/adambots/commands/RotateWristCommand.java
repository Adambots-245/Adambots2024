// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateWristCommand extends Command {
  /** Creates a new RotateWristCommand. */
  private ArmSubsystem armSubsystem;
  double wristIncrement;
  boolean manualAdjust;
  public RotateWristCommand(ArmSubsystem armSubsystem, double wristIncrement, boolean manualAdjust) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.wristIncrement = wristIncrement;
    this.manualAdjust = manualAdjust;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.incrementWristAngle(wristIncrement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !manualAdjust;
  }
}
