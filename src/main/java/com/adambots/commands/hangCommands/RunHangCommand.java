// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.hangCommands;

import com.adambots.subsystems.HangSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RunHangCommand extends Command {
  private HangSubsystem hangSubsystem;
  private double speed;
  private int inc = 0;

  public RunHangCommand(HangSubsystem hangSubsystem, double speed) {
    addRequirements(hangSubsystem);

    this.hangSubsystem = hangSubsystem;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (speed > 0) {
      hangSubsystem.setSolenoids(true); //Engage solenoids if we are running in the direction they are needed.
    }
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inc++;

    if (inc >= 15) { //Wait for the solenoids to engage before moving
      hangSubsystem.setLeftMotorSpeed(speed);
      hangSubsystem.setRightMotorSpeed(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangSubsystem.setLeftMotorSpeed(0);
    hangSubsystem.setRightMotorSpeed(0);

    // hangSubsystem.setSolenoids(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
