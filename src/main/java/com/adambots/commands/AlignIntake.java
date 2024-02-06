// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.adambots.subsystems.IntakeSubsystem;

public class AlignIntake extends Command {
  private IntakeSubsystem intakeSubsystem;
  /** Creates a new RunIntakeCommand. */
  public AlignIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setGroundIntakeMotorSpeed(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isPieceInRobot() == false){
      intakeSubsystem.setGroundIntakeMotorSpeed(0);
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isPieceInRobot();
  }
}
