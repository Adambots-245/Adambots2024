// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootWhenAligned extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private int inc;
  private boolean increment;

  public ShootWhenAligned(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addRequirements(intakeSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
    increment = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.isAtTargetSpeed() && armSubsystem.getCurrentState() == ArmConstants.speakerState
        && armSubsystem.isAtTargetState() && VisionHelpers.isAligned(VisionConstants.aprilLimelite, 3)
        && VisionHelpers.getAprilHorizDist() < 3) {
      intakeSubsystem.setMotorSpeed(1);
      increment = true;
    }
    if (increment) {
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
    shooterSubsystem.setTargetWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 15;
  }
}
