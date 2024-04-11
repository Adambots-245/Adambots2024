// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.ShooterConstants;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class SpinFlywheelsCommand extends Command {
  /** Creates a new SpinFlywheelsCommand. */
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public SpinFlywheelsCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    // addRequirements(shooterSubsystem);

    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!intakeSubsystem.getLockOut()) {
      shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeSubsystem.getLockOut();
  }
}
