// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ShooterConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;

import edu.wpi.first.wpilibj2.command.Command;


public class AutonIntakeCommand extends Command {
  /** Creates a new FloorIntakeCommand. */
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private CANdleSubsystem candle;

  private State shootState;

  public AutonIntakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, CANdleSubsystem candle, State shootState) {
    addRequirements(armSubsystem, intakeSubsystem, shooterSubsystem);

    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.candle = candle;
    this.shootState = shootState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setCurrentState(ArmConstants.floorState);
    intakeSubsystem.setMotorSpeed(0.17);
    candle.setAnimation(AnimationTypes.Larson);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setMotorSpeed(0.06);
      armSubsystem.setCurrentState(shootState);
      candle.setAnimation(AnimationTypes.Fire);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
    armSubsystem.setCurrentState(shootState);

    shooterSubsystem.setTargetWheelSpeed(ShooterConstants.lowSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
