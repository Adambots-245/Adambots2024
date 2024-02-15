// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.adambots.Constants.ArmConstants.State;
import com.adambots.subsystems.ArmSubsystem;


public class ChangeArmStateCommand extends Command {
  /** Creates a new ChangeArmStateCommand. */
  ArmSubsystem armSubsystem;
  State armState;
  double wristAngle;
  double shoulderAngle;

  public ChangeArmStateCommand(ArmSubsystem armSubsystem, State armState) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.armState = armState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristAngle = armState.getWristAngle();
    shoulderAngle = armState.getShoulderAngle();
    armSubsystem.setWristAngle(wristAngle);
    armSubsystem.setShoulderAngle(shoulderAngle);

    armSubsystem.setCurrentState(armState.getStateName());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
