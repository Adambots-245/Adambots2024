// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.CANdleSubsystem.AnimationTypes;


public class IntakeWithAdjustCommand extends Command {
  /** Creates a new IntakeWithAdjustCommand. */
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  CANdleSubsystem caNdleSubsystem;
  ShooterSubsystem shooterSubsystem;

  private String state = "initial";
  private int inc = 0;

  public IntakeWithAdjustCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, CANdleSubsystem caNdleSubsystem) {
    addRequirements(armSubsystem, intakeSubsystem);

    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.caNdleSubsystem = caNdleSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!intakeSubsystem.isSecondPieceInRobot()) {
      armSubsystem.setCurrentState(ArmConstants.floorState);
      intakeSubsystem.setGroundIntakeMotorSpeed(0.2);
    }
    shooterSubsystem.setTargetWheelSpeed(0);
    state = "initial";
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(inc);
    if (state == "initial" && intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setGroundIntakeMotorSpeed(0.12);
      armSubsystem.incrementWristAngle(15);
      state = "touchNote"; //keep this line, prevents above code from running repeatedly
      caNdleSubsystem.setOverride(true);
      caNdleSubsystem.setColor(LEDConstants.green);
    } if (state == "touchNote" && intakeSubsystem.isSecondPieceInRobot()) {
      state = "incrementing";
    }
    if (state == "incrementing") {
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setCurrentState(ArmConstants.defaultState);
    intakeSubsystem.setGroundIntakeMotorSpeed(0);

    caNdleSubsystem.setOverride(false);
    caNdleSubsystem.setColor(LEDConstants.adambotsYellow);
    caNdleSubsystem.changeAnimation(AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 10;
  }
}
