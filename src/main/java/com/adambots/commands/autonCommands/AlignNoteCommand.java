// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.1, 0, 0.03);
  private int count;
  private final double filterSens = 0.1;
  private double oldHorizAngle;

  public AlignNoteCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;

    Dash.add("horizAngle", () -> oldHorizAngle);
  }


  @Override
  public void initialize() {
    count = 0;
    oldHorizAngle = VisionHelpers.getHorizAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizAngle = filterSens*VisionHelpers.getHorizAngle() + (1-filterSens)*oldHorizAngle;
    oldHorizAngle = horizAngle;

    double drive_output = m_turningPIDController.calculate(horizAngle, 0);
    driveTrainSubsystem.drive(0, drive_output, 0, false);
    if (VisionHelpers.isAligned()) {
      count++;
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
    return count >= 2;
  }
}
