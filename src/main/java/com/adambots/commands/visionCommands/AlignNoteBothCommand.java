package com.adambots.commands.visionCommands;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

// CURRENTLY NOT WORKING
public class AlignNoteBothCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController vertPIDController = new PIDController(VisionConstants.kpBothPID, 0, VisionConstants.kdBothPID);
  private final PIDController horizPIDController = new PIDController(VisionConstants.kpBothPID, 0, VisionConstants.kdBothPID);
  private int vertCount;
  private int horizCount;
  private int notDetectedCount;
  private final double filterSens = 0.1;
  private double oldHoriz;
  private double newHoriz;
  private double oldVert;
  private double newVert;
  private double horizOutput;
  private double vertOutput;

  
  public AlignNoteBothCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
  }


  @Override
  public void initialize() {
    newHoriz = 0;
    newVert = 0;
    horizCount = 0;
    vertCount = 0;
    notDetectedCount = 0;
    horizOutput = 0;
    vertOutput = 0;
    oldHoriz = VisionHelpers.getHorizAngle(VisionConstants.noteLimelite);
    oldVert = VisionHelpers.getGamePieceArea(VisionConstants.noteLimelite);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newVert = filterSens*VisionHelpers.getGamePieceArea(VisionConstants.noteLimelite) + (1-filterSens)*oldVert;
    newHoriz = filterSens*VisionHelpers.getHorizAngle(VisionConstants.noteLimelite) + (1-filterSens)*oldHoriz;
    oldHoriz = newHoriz;
    oldVert = newVert;
    horizOutput = horizPIDController.calculate(newHoriz, 0);
    vertOutput = vertPIDController.calculate(newVert, 24);
    if (VisionHelpers.isDetected(VisionConstants.noteLimelite) == true){
      if (vertCount >= 1){
        driveTrainSubsystem.drive(0, horizOutput, 0, false);
      } else if (horizCount >= 3){
        driveTrainSubsystem.drive(vertOutput, 0, 0, false);
      } else {
        driveTrainSubsystem.drive(vertOutput, horizOutput, 0, false);
      }
    } else {
      driveTrainSubsystem.stop();
    }
    if(newVert > 24){
      vertCount++; 
    }
    if(Math.abs(newHoriz) < 2 && VisionHelpers.isDetected(VisionConstants.noteLimelite)){
      horizCount++;
    }
    if (!VisionHelpers.isDetected(VisionConstants.noteLimelite)) {
      notDetectedCount++;
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
    return (vertCount >= 1 && horizCount >= 1) || notDetectedCount >= 50;
  }
}
