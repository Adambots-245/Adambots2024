// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.VisionHelpers;


public class FloorIntakeAndShootCommand extends Command {
  /** Creates a new IntakeWithAdjustCommand. */
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;

  private String state = "initial";

  public FloorIntakeAndShootCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(armSubsystem, intakeSubsystem, shooterSubsystem);

    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!intakeSubsystem.isSecondPieceInRobot()) {
      armSubsystem.setCurrentState(ArmConstants.floorState);
      intakeSubsystem.setGroundIntakeMotorSpeed(0.17);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (state == "initial" && intakeSubsystem.isFirstPieceInRobot()) {
    //   intakeSubsystem.setGroundIntakeMotorSpeed(0.18);
    //   armSubsystem.incrementWristAngle(30);
    //   state = "touchNote"; //keep this line, prevents above code from running repeatedly
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setCurrentState(ArmConstants.floorShootState);

    intakeSubsystem.setGroundIntakeMotorSpeed(0);
    VisionHelpers.blinkLight(VisionConstants.noteLimelite);
    shooterSubsystem.setWheelSpeed(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSecondPieceInRobot();
  }
}
