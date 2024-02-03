// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlignNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.1, 0, 0.02);
  private int count;
  private int notDetected;
  private final double filterSens = 0.1;
  private double oldHorizAngle;
  public double drive_output;
  private double horizAngle;

  public AlignNoteCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    // Dash.add("horizFliteredAngle", () -> horizAngle);
  }
  public double getDriveOutput(){
    return drive_output;
  }

  @Override
  public void initialize() {
    horizAngle = 0;
    count = 0;
    notDetected = 0;
    drive_output = 0;
    oldHorizAngle = VisionHelpers.getHorizAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    horizAngle = filterSens*VisionHelpers.getHorizAngle() + (1-filterSens)*oldHorizAngle;
    oldHorizAngle = horizAngle;

    drive_output = m_turningPIDController.calculate(horizAngle, 0);
    if (VisionHelpers.getHorizAngle() != 0){
      if (VisionHelpers.getGamePieceArea() < 18){
        driveTrainSubsystem.drive(1, drive_output, 0, false);
      } else {
        driveTrainSubsystem.drive(0, drive_output, 0, false);
      }
    }
    if((horizAngle>-2&&horizAngle<2)&&VisionHelpers.isDetected()){
      count++;
    }
    if (VisionHelpers.isDetected() != false) {
      notDetected++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
      // driveTrainSubsystem.drive(0.2, 0, 0, false);
      // new WaitCommand(0.1);
      driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 1 || notDetected >= 50;
  }
}
