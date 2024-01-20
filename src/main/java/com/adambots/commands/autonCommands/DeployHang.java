// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.subsystems.HangSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployHang extends Command {
  
  HangSubsystem hangSubsystem;
  /** Creates a new DeployHang. */
  public DeployHang(HangSubsystem hangSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangSubsystem);
    this.hangSubsystem = hangSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangSubsystem.setLeftHangMotorSpeed(0.1);
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
