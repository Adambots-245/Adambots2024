// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.armCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PrimeShooterCommandFeed extends Command {
  /** Creates a new FeedShooterCommand. */
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private double shooterSpeed;
  private IntakeSubsystem intakeSubsystem;
  private boolean finished;
  
  public PrimeShooterCommandFeed(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double shooterSpeed) {
    addRequirements(armSubsystem, shooterSubsystem);
    
    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterSpeed = shooterSpeed;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intakeSubsystem.getLockOut() && !armSubsystem.getCurrentStateName().equals("closeFloorShoot")) {
      armSubsystem.setCurrentState(ArmConstants.defaultState);
      shooterSubsystem.setTargetWheelSpeed(shooterSpeed);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
