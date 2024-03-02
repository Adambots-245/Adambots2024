// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ShooterConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;

import edu.wpi.first.wpilibj2.command.Command;


public class FloorIntakeCommand extends Command {
  /** Creates a new IntakeWithAdjustCommand. */
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  CANdleSubsystem candle;

  private State shootState;

  // private String state = "initial";

  public FloorIntakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, CANdleSubsystem candle, State shootState) {
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
    intakeSubsystem.setGroundIntakeMotorSpeed(0.17);
    candle.clearAllAnims();
    candle.changeAnimation(AnimationTypes.Larson);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.1);
      armSubsystem.setCurrentState(shootState);
      candle.clearAllAnims();
      candle.changeAnimation(AnimationTypes.Fire);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setGroundIntakeMotorSpeed(0);
    armSubsystem.setCurrentState(shootState);

    shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed);

    // candle.clearAllAnims();
    // candle.changeAnimation(AnimationTypes.Strobe);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
