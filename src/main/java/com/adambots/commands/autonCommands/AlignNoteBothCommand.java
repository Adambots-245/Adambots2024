package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

// CURRENTLY NOT WORKING
public class AlignNoteBothCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private final PIDController yPIDController = new PIDController(0.2, 0, 0.05);
  private final PIDController xPIDController = new PIDController(0.1, 0, 0.04);
  private int count;
  private int notDetected;
  private final double filterSens = 0.1;
  private double oldX;
  private double oldY;

  public AlignNoteBothCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
  }


  @Override
  public void initialize() {
    count = 0;
    notDetected = 0;
    oldY = VisionHelpers.getHorizAngle();
    oldX = VisionHelpers.getGamePieceArea();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newX = filterSens*VisionHelpers.getGamePieceArea() + (1-filterSens)*oldY;
    oldX = newX;
    double newY = filterSens*VisionHelpers.getHorizAngle() + (1-filterSens)*oldX;
    oldY = newY;

    double drive_output_y = xPIDController.calculate(newX, 0);
    double drive_output_x = yPIDController.calculate(newY, 10);
    driveTrainSubsystem.drive(drive_output_x, drive_output_y, 0, false);
    if (VisionHelpers.isAligned() && VisionHelpers.isDistanceAligned()) {
      count++;
    }
    if (VisionHelpers.isDetected() != false) {
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
    return count >= 1 || notDetected >= 50;
  }
}
