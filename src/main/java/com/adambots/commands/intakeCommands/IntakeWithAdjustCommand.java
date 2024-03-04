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
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private CANdleSubsystem caNdleSubsystem;
  private ShooterSubsystem shooterSubsystem;

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
      intakeSubsystem.setMotorSpeed(0.2);
    }
    shooterSubsystem.setTargetWheelSpeed(0);
    state = "initial";
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == "initial" && intakeSubsystem.isFirstPieceInRobot()) {
      intakeSubsystem.setMotorSpeed(0.12);
      armSubsystem.incrementWristAngle(15);
      state = "touchNote"; //keep this line, prevents above code from running repeatedly
      caNdleSubsystem.setColor(LEDConstants.green);
    } if (intakeSubsystem.isSecondPieceInRobot()) {
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
    intakeSubsystem.setMotorSpeed(0);

    caNdleSubsystem.setColor(LEDConstants.yellow);
    caNdleSubsystem.setAnimation(AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 10;
  }
}
