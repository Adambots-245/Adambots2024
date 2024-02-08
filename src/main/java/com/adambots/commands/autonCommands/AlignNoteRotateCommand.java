package com.adambots.commands.autonCommands;
import com.adambots.Constants.DriveConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignNoteRotateCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController m_turningPIDController = new PIDController(0.1, 0, 0.032);
  private int alignedCount;
  private int notDetectedCount;
  private final double filterSens = 0.1;
  private double oldRotate;
  private double newRotate;
  private double drive_output;
  private boolean fieldOrientated;
  private boolean continuous;

  public AlignNoteRotateCommand(DrivetrainSubsystem driveTrainSubsystem, boolean fieldOrientated, boolean continuous) {
    addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.fieldOrientated = fieldOrientated;
    this.continuous = continuous;
  }

  @Override
  public void initialize() {
    oldRotate = VisionHelpers.getHorizAngle();;
    alignedCount = 0;
    notDetectedCount = 0;
    drive_output = 0;
    newRotate = VisionHelpers.getHorizAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Filter to stop large changes in data while runing, to make the PID work better
    newRotate = filterSens*VisionHelpers.getHorizAngle() + (1-filterSens)*oldRotate;
    oldRotate = newRotate;

    // Calculates the drive rotation
    drive_output = m_turningPIDController.calculate(Math.abs(newRotate), 0);
    //Checks to see if we have an object detected
    if (VisionHelpers.isDetected()){
      //Aligns differntly if it is field orientated or not
      if (fieldOrientated == true){
          //Moves left or right depending on the angle
          if (newRotate > 0){
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , drive_output, true);
          } else {
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , -drive_output, true);
          }   
      } else {
          //Moves left or right depending on the angle
          if (newRotate > 0){
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , drive_output, false);
          } else {
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , -drive_output, false);
          }  
      }
      
    } else {
      driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , -Buttons.rotateSupplier.getAsDouble()*DriveConstants.kTeleopRotationalSpeed, true);
    }
    //Checks to see if the filtered angle is within the aligned bounds
    //Checks to see if the robot is at that position for more than just a single moment
    if((newRotate>-5&&newRotate<5)&&VisionHelpers.isDetected()){
      alignedCount++;
    }
    //If the robot is not detecting a piece for a while, it adds to this counter
    if (VisionHelpers.isDetected() == false) {
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
    if (continuous == false){
      return alignedCount >= 10 || notDetectedCount >= 50;
    } else {
      return false;
    }
  }
}
