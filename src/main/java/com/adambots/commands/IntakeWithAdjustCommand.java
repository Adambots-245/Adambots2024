// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.commands.autonCommands.AdjustNoteCommand;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;


public class IntakeWithAdjustCommand extends Command {
  /** Creates a new ChangeArmStateCommand. */
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;

  private String state = "initial";

  public IntakeWithAdjustCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(armSubsystem, intakeSubsystem);

    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!intakeSubsystem.isSecondPieceInRobot()) {
      armSubsystem.setCurrentState(ArmConstants.floorState);
      intakeSubsystem.setGroundIntakeMotorSpeed(0.3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == "initial" && intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.09);
      armSubsystem.incrementWristAngle(10);
      state = "touchNote"; //keep this line, prevents above code from running repeatedly
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setCurrentState(ArmConstants.defaultState);
    intakeSubsystem.setGroundIntakeMotorSpeed(0);
    new AdjustNoteCommand(intakeSubsystem).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
