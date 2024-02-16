// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunIntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double groundIntakeMotorSpeed;
  private double slowIntakeSpeed;
  private ArmSubsystem armSubsystem;

  /** Creates a new RunIntakeCommand. */
  public RunIntakeCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, double groundIntakeMotorSpeed, double slowIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, armSubsystem);
  
    this.intakeSubsystem = intakeSubsystem;
    this.groundIntakeMotorSpeed = groundIntakeMotorSpeed;
    this.slowIntakeSpeed = slowIntakeSpeed;
    this.armSubsystem = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setGroundIntakeMotorSpeed(groundIntakeMotorSpeed);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(slowIntakeSpeed);
      new SequentialCommandGroup(
      new RotateWristCommand(armSubsystem, 0.5, false),
      new WaitCommand(0.3)
      ).schedule();
    }
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