package com.adambots.commands.visionCommands;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAprilCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController noteTurningPIDController = new PIDController(VisionConstants.kPThetaController, 0, VisionConstants.kDThetaController);

  private String limelight;
  private int inc;
  

  public RotateToAprilCommand(DrivetrainSubsystem driveTrainSubsystem, String limelight) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelight;
  }

  @Override
  public void initialize() {
    inc = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive_output = noteTurningPIDController.calculate(Math.toRadians(VisionHelpers.getHorizAngle(limelight)), 0);

    driveTrainSubsystem.drive(0, 0, drive_output, true);

    inc++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc >= 50;
  }
}
