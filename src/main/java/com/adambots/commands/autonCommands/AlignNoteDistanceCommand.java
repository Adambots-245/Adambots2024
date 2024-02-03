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

public class AlignNoteDistanceCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.1, 0, 0.02);
  private int count;
  private int notDetected;
  private final double filterSens = 0.1;
  private double oldDistance;
  double distance;
  

  public AlignNoteDistanceCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    // Dash.add("distance", () -> distance);
  }


  @Override
  public void initialize() {
    count = 0;
    notDetected = 0;
    oldDistance = VisionHelpers.getGamePieceArea();
    // driveTrainSubsystem.drive(0.2, 0, 0, false);
    // new WaitCommand(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = filterSens*VisionHelpers.getGamePieceArea() + (1-filterSens)*oldDistance;
    oldDistance = distance;

    double drive_output = m_turningPIDController.calculate(distance, 26);
    if (VisionHelpers.getGamePieceArea() > 0){
      driveTrainSubsystem.drive(drive_output, 0, 0, false);
    }
    if(distance>24&&distance<27){
      count++; 
      System.out.print("distance: ");
      System.out.println(distance);
    }
    // if (VisionHelpers.isDistanceAligned()) {
    //   count++;
    // }
    if (VisionHelpers.isDetected() != false) {
      notDetected++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
      driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 3 || notDetected >= 25;
  }
}