// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.ShooterConstants;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class AdjustNoteCommand extends Command {
  /** Creates a new AdjustNoteCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private int state;
  private int inc;
  private int timeOut;
  private boolean finished;

  public AdjustNoteCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    inc = 0;
    timeOut = 0;
    finished = false;
    intakeSubsystem.setLockOut(true); //Prevent going to shoot state while still adjusting
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timeOut++;

    // if (state == 0 && inc <= 25) {
    //   intakeSubsystem.setMotorSpeed(0.25); //Intake for 15 ticks
    // } else if (state == 0 && inc > 25) {
    //   state = 1;
    //   intakeSubsystem.setMotorSpeed(-IntakeConstants.lowSpeed); //Outtake until sensor
    // } else if (state == 1 && intakeSubsystem.isSecondPieceInRobot()) {
    //   inc = 0;
    //   state = 2;
    //   intakeSubsystem.setMotorSpeed(IntakeConstants.lowSpeed); //Intake for 15 ticks
    // } else if (state == 2 && inc > 15) {
    //   state = 3;
    //   intakeSubsystem.setMotorSpeed(-IntakeConstants.lowSpeed); //Outtake until sensor
    // } else if (state == 3 && intakeSubsystem.isSecondPieceInRobot()) {
    //   inc = 0;
    //   state = 4;
    //   intakeSubsystem.setMotorSpeed(0.0); //Stop
    //   finished = true;
    // }

    if (shooterSubsystem.getShooterVelocity() < 0) {
      inc++;

      if (state == 0 && inc <= 25) {
        intakeSubsystem.setMotorSpeed(0.25); //Intake for 15 ticks
      } else if (state == 0 && inc > 25) {
        state = 1;
        intakeSubsystem.setMotorSpeed(-0.08); //Outtake until sensor
      } else if (state == 1 && intakeSubsystem.isSecondPieceInRobot()) {
        inc = 0;
        state = 4;
        intakeSubsystem.setMotorSpeed(0); //Intake for 15 ticks
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setLockOut(false); //Ensure this is set back to false to allow going to shoot state
    intakeSubsystem.setMotorSpeed(0);
    shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || timeOut > 75;
  }
}
