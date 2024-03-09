// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.armCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RetractShooterCommand extends Command {
  /** Creates a new RetractShooterCommand. */
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  
  public RetractShooterCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(armSubsystem, shooterSubsystem);
    
    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setCurrentState(ArmConstants.defaultState);
    shooterSubsystem.setTargetWheelSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

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
