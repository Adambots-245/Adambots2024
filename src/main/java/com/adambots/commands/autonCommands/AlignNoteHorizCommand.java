package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteHorizCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.1, 0, 0.02);
  private int alignedCount;
  private int notDetectedCount;
  private final double filterSens = 0.1;
  private double oldHoriz;
  private double newHoriz;
  private double drive_output;

  public AlignNoteHorizCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    System.out.println();
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  @Override
  public void initialize() {
    newHoriz = 0;
    alignedCount = 0;
    notDetectedCount = 0;
    drive_output = 0;
    oldHoriz = VisionHelpers.getHorizAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Filter to stop large changes in data while runing, to make the PID work better
    newHoriz = filterSens*VisionHelpers.getHorizAngle() + (1-filterSens)*oldHoriz;
    oldHoriz = newHoriz;

    // Calculates the drive output/speed
    drive_output = m_turningPIDController.calculate(newHoriz, 0);
    //Checks to see if we have an object detected
    if (VisionHelpers.isDetected()){
      //Only moves the robot sidewards and forwards at the same time if the gamepiece isn't too close
      //Otherwise it just aligns to the side 
      // if (VisionHelpers.getGamePieceArea() < 18){
      //   driveTrainSubsystem.drive(1, drive_output, 0, false);
      // } else {
      //   driveTrainSubsystem.drive(0, drive_output, 0, false);
      // }

      driveTrainSubsystem.drive(0, drive_output, driveTrainSubsystem.getRot(), false);
    }
    //Checks to see if the filtered angle is within the aligned bounds
    //Checks to see if the robot is at that position for more than just a single moment
    if((newHoriz>-2&&newHoriz<2)&&VisionHelpers.isDetected()){
      alignedCount++;
    }
    //If the robot is not detecting a piece for a while, it adds to this counter
    if (VisionHelpers.isDetected() != false) {
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
    //If the robot is aligned for some time, or the robot is not detecting a piece the command ends
    return alignedCount >= 1 || notDetectedCount >= 50;
  }
}
