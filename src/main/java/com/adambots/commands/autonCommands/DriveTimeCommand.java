// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveTimeCommand extends Command {
  
  private DrivetrainSubsystem driveTrainSubsystem;
  private double timeSec;
  private double inc;
  private double x;
  private double y;
  private double rot;
  private Boolean relative;

  public DriveTimeCommand(DrivetrainSubsystem driveTrainSubsystem, double x, double y, double rot, Boolean relative, double timeSec) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.x = x;
    this.y = y;
    this.rot = rot;
    this.relative = relative;
    this.timeSec = timeSec;
  }


  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.drive(x, y, rot, relative);
    inc++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc >= timeSec*50;
  }
}
