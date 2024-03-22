// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.visionCommands;

import com.adambots.Constants.VisionConstants;
import com.adambots.Robot;
import com.adambots.RobotMap;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionOdomResetCommand extends Command {
  /** Creates a new VisionOdomResetCommand. */
  DrivetrainSubsystem driveTrainSubsystem;
  private PIDController fakePIPidController = new PIDController(0, 0, 0);

  public VisionOdomResetCommand(DrivetrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    fakePIPidController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //TODO: Add support for red side
    //Calculate angle to speaker
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite) && VisionHelpers.getAprilTagBotPose2dBlue() != null) {
      if (VisionHelpers.getAprilTagBotPose2dBlue().getY() > 5) {
        // System.out.println(VisionHelpers.getAprilTagBotPose2dBlue().getY());
        double gyroYaw = RobotMap.gyro.getContinuousYawRad();
        double aprilYaw = (VisionHelpers.getAprilTagBotPose2dBlue().getRotation().getRadians() + Math.PI) ;
        if (Robot.isOnRedAlliance()) {
          aprilYaw = (VisionHelpers.getAprilTagBotPose2dBlue().getRotation().getRadians()) ;
        }
        fakePIPidController.calculate(aprilYaw, gyroYaw);

        if (VisionHelpers.getAprilHorizDist() < 3.5 && Math.abs(fakePIPidController.getPositionError()) < Math.toRadians(7.5)){
          // System.out.print("updated");
          driveTrainSubsystem.resetOdometryXY(VisionHelpers.getAprilTagBotPose2dBlue());
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
