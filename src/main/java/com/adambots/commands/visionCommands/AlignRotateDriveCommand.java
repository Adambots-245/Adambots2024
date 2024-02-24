package com.adambots.commands.visionCommands;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignRotateDriveCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem caNdleSubsystem;
  private final PIDController noteTurningPIDController = new PIDController(VisionConstants.kPNoteThetaController, 0, VisionConstants.kDNoteThetaController);
  private final PIDController aprilTurningPIDController = new PIDController(VisionConstants.kPAprilThetaController, 0, VisionConstants.kDAprilThetaController);
  private double drive_output;
  private boolean fieldOrientated;
  private String limelight;
  

  public AlignRotateDriveCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem caNdleSubsystem, boolean fieldOrientated, String limelight) {
    addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.caNdleSubsystem = caNdleSubsystem;
    this.fieldOrientated = fieldOrientated;
    this.limelight = limelight;
  }

  @Override
  public void initialize() {
    drive_output = 0;
    caNdleSubsystem.changeAnimation(CANdleSubsystem.AnimationTypes.SetAll);
    caNdleSubsystem.setColor(LEDConstants.yellow);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = VisionHelpers.getHorizAngle(limelight);

    // Calculates the drive rotation
    if (limelight == VisionConstants.noteLimelite){
      drive_output = noteTurningPIDController.calculate((double)(Math.abs(Math.toRadians(rotate))), 0);
    } else{
      drive_output = aprilTurningPIDController.calculate((double)(Math.abs(Math.toRadians(rotate))), 0);
    }

    //Checks to see if we have an object detected
    if (VisionHelpers.isDetected(limelight)){
      //Aligns differntly if it is field orientated or not
      if (fieldOrientated == true){
          //Moves left or right depending on the angle
          if (rotate > 0){
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , drive_output, true);
          } else {
            driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , -drive_output, true);
          }   
      } else {
          //Moves left or right depending on the angle
          if (rotate > 0){
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
    if((Math.abs(rotate)<5)&&VisionHelpers.isDetected(limelight)){
      caNdleSubsystem.setColor(LEDConstants.blue);
    } else {
      caNdleSubsystem.setColor(LEDConstants.yellow);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    caNdleSubsystem.clearAllAnims();
    caNdleSubsystem.changeAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
