package com.adambots.commands.visionCommands;
import com.adambots.Robot;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem caNdleSubsystem;
  private final PIDController pidController = new PIDController(VisionConstants.kPTranslateController, 0, VisionConstants.kDTranslateController);
  private double drive_output;

  public DriveToNoteCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem caNdleSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.caNdleSubsystem = caNdleSubsystem;
  }

  @Override
  public void initialize() {
    caNdleSubsystem.setColor(LEDConstants.red);
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (VisionHelpers.isDetected(VisionConstants.noteLimelite)){
      drive_output = pidController.calculate(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite), 0);

      if (Robot.isOnRedAlliance()) {
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
      caNdleSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
