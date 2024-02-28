package com.adambots.commands.visionCommands;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem caNdleSubsystem;
  private final PIDController translatePIDController = new PIDController(VisionConstants.kPNoteThetaController, 0, VisionConstants.kDNoteThetaController);
  private double drive_output;

  public DriveToNoteCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem caNdleSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.caNdleSubsystem = caNdleSubsystem;
   
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
    double strafe = VisionHelpers.getHorizAngle(VisionConstants.noteLimelite);

    // Calculates the drive rotation
    drive_output = translatePIDController.calculate(Math.toRadians(strafe), 0);
    
    var alliance = DriverStation.getAlliance();
    if (VisionHelpers.isDetected(VisionConstants.noteLimelite) && alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red) {
        driveTrainSubsystem.drive(-0.5, -drive_output, 0, true);
      } else {
        driveTrainSubsystem.drive(0.5, drive_output, 0, true);
      }
    } else {
      driveTrainSubsystem.stop();
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
