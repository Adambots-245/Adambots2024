package com.adambots.commands.visionCommands;

import com.adambots.Robot;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAprilCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem ledSubsystem;
  private final PIDController turningPIDController = new PIDController(VisionConstants.kPThetaController, 0,
      VisionConstants.kDThetaController);
  private int alignedCount;
  private double drive_output;
  private double offset;

  public RotateToAprilCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem caNdleSubsystem,
      double offset) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ledSubsystem = caNdleSubsystem;
    this.offset = offset;
  }

  @Override
  public void initialize() {
    alignedCount = 0;
    drive_output = 0;
    ledSubsystem.setColor(LEDConstants.red);

    if (Robot.isOnRedAlliance()) {
      offset = -offset;
    }
    ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = VisionHelpers.getHorizAngle(VisionConstants.aprilLimelite);

    // Calculates the drive rotation
    drive_output = turningPIDController.calculate(Math.toRadians(rotate), Math.toRadians(offset));

    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)
        && (VisionHelpers.getAprilTagID() == 4 || VisionHelpers.getAprilTagID() == 7)) {
      driveTrainSubsystem.drive(0, 0, drive_output, true);
    }

    // Checks to see if the filtered angle is within the aligned bounds
    // Checks to see if the robot is at that position for more than just a single
    // moment
    if (Math.abs(turningPIDController.getPositionError()) < Math.toRadians(1)) {
      alignedCount++;
      ledSubsystem.setColor(LEDConstants.green);
    } else {
      ledSubsystem.setColor(LEDConstants.red);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    ledSubsystem.clearAllAnims();
    ledSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the robot is aligned for some time, or the robot is not detecting a piece
    // the command ends
    return alignedCount >= 5;
  }
}
