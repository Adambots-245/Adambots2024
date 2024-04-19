package com.adambots.commands.visionCommands;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.RobotMap;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWhileDrivingCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private PIDController turningPIDController = new PIDController(VisionConstants.kPThetaController, 0, VisionConstants.kDThetaController);
  private double rotation_output;
  private String limelight;
  private double rotate;

  public AlignWhileDrivingCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem ledSubsystem, String limelight) {
    addRequirements(driveTrainSubsystem);

    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelight;
  }

  @Override
  public void initialize() {
    // candleSubsystem.setColor(LEDConstants.yellow);
    rotate = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (VisionHelpers.isDetected(limelight)){
      // rotate = VisionHelpers.getAprilTagBotPose2d().getRotation().getRadians();
      rotate = -Math.toRadians(VisionHelpers.getHorizAngle(limelight)) + RobotMap.gyro.getContinuousYawRad();


    }
    // Calculates the drive rotation
    if (limelight == VisionConstants.noteLimelite) {
      rotation_output = turningPIDController.calculate(Math.toRadians(rotate), 0);
    } else if (limelight == VisionConstants.defaultAprilLimelite){
        // && (VisionHelpers.getAprilTagID() == 4 || VisionHelpers.getAprilTagID() == 7)) {
          rotation_output = turningPIDController.calculate(RobotMap.gyro.getContinuousYawRad(), rotate);
    } else {
      rotation_output = 0;
    }

     driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
          Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, rotation_output, true);
    // Checks to see if we have an object detected
    if (VisionHelpers.isDetected(limelight)) {
      // driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
      //     Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, rotation_output, true);
    } else {
      // driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
      //     Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
      //     Buttons.rotateSupplier.getAsDouble() * DriveConstants.kTeleopRotationalSpeed, true);
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
    // if (limelight == VisionConstants.noteLimelite) {
    //   return true;
    // }
    return false;
  }
}
