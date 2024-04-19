// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class AutonIntakeCommand extends Command {
  /** Creates a new AutonIntakeCommand. */
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private CANdleSubsystem candle;

  public AutonIntakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, CANdleSubsystem candle) {
    addRequirements(armSubsystem, intakeSubsystem);

    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.candle = candle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " + Initializing");
    armSubsystem.setCurrentState(ArmConstants.floorState);
    intakeSubsystem.setMotorSpeed(IntakeConstants.intakeSpeed);
    candle.setColor(LEDConstants.orange);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setMotorSpeed(IntakeConstants.lowSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
    candle.setColor(LEDConstants.adambotsYellow);
    candle.setAnimation(AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
