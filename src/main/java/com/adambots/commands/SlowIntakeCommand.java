// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.adambots.subsystems.IntakeSubsystem;

public class SlowIntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double newSlowSpeed;
  /** Creates a new RunIntakeCommand. */
  public SlowIntakeCommand(IntakeSubsystem intakeSubsystem, double newSlowSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.newSlowSpeed = newSlowSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intakeSubsystem.setGroundIntakeMotorSpeed(newSlowSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setGroundIntakeMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
