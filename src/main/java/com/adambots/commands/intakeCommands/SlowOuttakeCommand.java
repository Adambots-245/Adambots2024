// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import com.adambots.subsystems.IntakeSubsystem;

public class SlowOuttakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private int inc = 0;

  public SlowOuttakeCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
    // if (!intakeSubsystem.isSecondPieceInRobot()) {
    intakeSubsystem.setMotorSpeed(-0.08);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inc++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 7 || intakeSubsystem.isSecondPieceInRobot() || intakeSubsystem.isFirstPieceInRobot();
  }
}
