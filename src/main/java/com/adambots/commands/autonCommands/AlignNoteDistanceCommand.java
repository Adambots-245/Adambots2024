// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteDistanceCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.5, 0, 0);
  private int count;
  private int notDetected;
  private final double filterSens = 0.1;
  private double oldDistance;
  

  public AlignNoteDistanceCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;

    //Dash.add("distanceFiltered", () -> oldDistance);
  }


  @Override
  public void initialize() {
    count = 0;
    notDetected = 0;
    oldDistance = VisionHelpers.getGamePieceArea();
    System.out.println("hi");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = filterSens*VisionHelpers.getGamePieceArea() + (1-filterSens)*oldDistance;
    oldDistance = distance;

    double drive_output = m_turningPIDController.calculate(distance, 0);
    driveTrainSubsystem.drive(-drive_output, 0, 0, false);
    if (VisionHelpers.isDistanceAligned()) {
      count++;
    }
    if (VisionHelpers.isDetected() != false) {
      notDetected++;
    }
  //   driveTrainSubsystem.drive(0, 0, 0, false);
  //  }
  //  if (VisionHelpers.isAligned()) {
  //   driveTrainSubsystem.drive(0, 0, 0, false);
  //  }
  //  else if (VisionHelpers.getHorizAngle() < 0 && VisionHelpers.isDetected()){
  //   driveTrainSubsystem.drive(0, 0.5, 0, false);
  //  }
  //  else{
  //   driveTrainSubsystem.drive(0, -0.5, 0, false);
  //  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 2 || notDetected == 50;
  }
}
