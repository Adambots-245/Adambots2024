package com.adambots.commands.visionCommands;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem ledSubsystem;
  private final PIDController pidController = new PIDController(VisionConstants.kPTranslateController, 0, VisionConstants.kDTranslateController);
  private double drive_output;

  public DriveToNoteCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem ledSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(LEDConstants.red);
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Huh? " + VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    // if (VisionHelpers.isDetected(VisionConstants.noteLimelite)){
      drive_output = pidController.calculate(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite), 0);

      // if (Robot.isOnRedAlliance()) {
      //   driveTrainSubsystem.drive(-0.5, -drive_output, 0, false);
      // } else {
        driveTrainSubsystem.drive(1, drive_output, 0, false);
      // }
    // } else {
    //   driveTrainSubsystem.stop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrainSubsystem.stop();
      ledSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
