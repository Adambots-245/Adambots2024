// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.ArmConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class HumanStationCommand extends Command {
  /** Creates a new FeedShooterCommand. */
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  
  public HumanStationCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(armSubsystem, intakeSubsystem);
    
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setCurrentState(ArmConstants.humanState);
    intakeSubsystem.setGroundIntakeMotorSpeed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isSecondPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0);
    } else if (intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.09);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setCurrentState(ArmConstants.defaultState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
