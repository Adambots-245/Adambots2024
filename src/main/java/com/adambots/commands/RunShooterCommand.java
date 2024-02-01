// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterCommand extends Command {
  /** Creates a new RunShooterCommand. */
  ShooterSubsystem shooterSubsystem;
  double shootMotorSpeed;

  public RunShooterCommand(ShooterSubsystem shooterSubsystem, double shootMotorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.shootMotorSpeed = shootMotorSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setWheelSpeed(shootMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setWheelSpeed(0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
