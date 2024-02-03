// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  public AlignNoteCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
  }


  @Override
  public void initialize() {
    System.out.println("hello");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (VisionHelpers.isAligned()) {
    driveTrainSubsystem.drive(0, 0, 0, isScheduled());
   }
   else if (VisionHelpers.getHorizAngle() < 0 && VisionHelpers.isDetected()){
    driveTrainSubsystem.drive(0.1, 0, 0, isScheduled());
   }
   else{
    driveTrainSubsystem.drive(-0.1, 0, 0, isScheduled());
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
