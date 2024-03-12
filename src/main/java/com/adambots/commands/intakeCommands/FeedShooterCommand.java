// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.ShooterConstants;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterCommand extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private int inc;
  private boolean increment;
  
  public FeedShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
    increment = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          // System.out.println("TRYINGGGGGGGGGGGGGGG");

    if (shooterSubsystem.isAtTargetSpeed()) {
      // System.out.println("TAGREY SPEEEEEEEEEEEED");

      intakeSubsystem.setMotorSpeed(IntakeConstants.shootSpeed);
      increment = true;
    }
    if(increment){
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
    shooterSubsystem.setTargetWheelSpeed(ShooterConstants.idleSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 15;
  }
}
