package com.adambots.commands.visionCommands.old;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteVertCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(VisionConstants.kpVertPID, 0, VisionConstants.kdVertPID);
  private int count;
  private int notDetected;
  private final double filterSens = 0.1;
  private double oldDistance;
  private double distance;
  

  public AlignNoteVertCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
  }


  @Override
  public void initialize() {
    count = 0;
    notDetected = 0;
    oldDistance = VisionHelpers.getGamePieceArea(VisionConstants.noteLimelite);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Filter to stop large changes in data while runing, to make the PID work better
    distance = filterSens*VisionHelpers.getGamePieceArea(VisionConstants.noteLimelite) + (1-filterSens)*oldDistance;
    oldDistance = distance;

    // Calculates the drive output/speed
    double drive_output = m_turningPIDController.calculate(distance, 25);
    if (VisionHelpers.isDetected(VisionConstants.noteLimelite) == true){
      // driveTrainSubsystem.drive(drive_output, 0, 0, false);

      driveTrainSubsystem.drive(drive_output, 0, 0, false);
    } else {
      driveTrainSubsystem.stop();
    }
    //Checks to see if the filtered area is within the aligned bounds
    //Checks to see if the robot is at that position for more than just a single moment
    if(distance>24&&distance<27){
      count++; 
    }
    //If the robot is not detecting a piece for a while, it adds to this counter
    if (VisionHelpers.isDetected(VisionConstants.noteLimelite) == false) {
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
    //If the robot is aligned for some time, or the robot is not detecting a piece the command ends
    return count >= 3 || notDetected >= 15;
  }
}