// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterCommand extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private double shooterSpeedThreshold;
  private int inc = 0;
  
  public FeedShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, double shooterSpeedThreshold) {
    addRequirements(intakeSubsystem, shooterSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterSpeedThreshold = shooterSpeedThreshold;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getShooterVelocity() >= shooterSpeedThreshold) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.3);
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setGroundIntakeMotorSpeed(0);
    shooterSubsystem.setWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 15;
  }
}
