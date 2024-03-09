// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.IntakeConstants;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class FancyAdjustCommand extends Command {
  /** Creates a new FancyAdjustCommand. */
  private IntakeSubsystem intakeSubsystem;
  private int state;
  private int inc;
  private boolean finished;

  public FancyAdjustCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    inc = 0;
    finished = false;
    intakeSubsystem.setLockOut(true); //Prevent going to shoot state while still adjusting
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inc++;

    if (state == 0 && inc <= 30) {
      intakeSubsystem.setMotorSpeed(IntakeConstants.lowSpeed); //Intake for 30 ticks
    } else if (state == 0 && inc > 30) {
      state = 1;
      intakeSubsystem.setMotorSpeed(-IntakeConstants.lowSpeed); //Outtake until sensor
    } else if (state == 1 && intakeSubsystem.isSecondPieceInRobot()) {
      inc = 0;
      state = 2;
      intakeSubsystem.setMotorSpeed(IntakeConstants.lowSpeed); //Intake for 30 ticks
    } else if (state == 2 && inc > 30) {
      state = 3;
      intakeSubsystem.setMotorSpeed(-IntakeConstants.lowSpeed); //Outtake until sensor
    } else if (state == 3 && intakeSubsystem.isSecondPieceInRobot()) {
      inc = 0;
      state = 4;
      intakeSubsystem.setMotorSpeed(0.0); //Stop
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setLockOut(false); //Ensure this is set back to false to allow going to shoot state
    intakeSubsystem.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
