package com.adambots.commands.visionCommands;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.LEDConstants;
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
  private CANdleSubsystem candleSubsystem;
  private PIDController turningPIDController = new PIDController(VisionConstants.kPThetaController, 0, VisionConstants.kDThetaController);
  private double rotation_output;
  private String limelight;
  private double rotate;

  public AlignWhileDrivingCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem ledSubsystem, String limelight) {
    addRequirements(driveTrainSubsystem);

    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.candleSubsystem = ledSubsystem;
    this.limelight = limelight;
  }

  @Override
  public void initialize() {
    candleSubsystem.setColor(LEDConstants.yellow);
    rotate = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (VisionHelpers.isDetected(limelight)){
      driveTrainSubsystem.resetOdometryXY(VisionHelpers.getAprilTagBotPose2dBlue());
      // rotate = VisionHelpers.getAprilTagBotPose2d().getRotation().getRadians();
      rotate = -Math.toRadians(VisionHelpers.getHorizAngle(limelight)) + RobotMap.gyro.getContinuousYawRad();


    }
    // Calculates the drive rotation
    if (limelight == VisionConstants.noteLimelite) {
      rotation_output = turningPIDController.calculate(Math.toRadians(rotate), 0);
    } else if (limelight == VisionConstants.aprilLimelite){
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

    // Checks to see if the angle is within the aligned bounds
    if (VisionHelpers.isDetected(limelight)) {
      if (Math.abs(rotate) < 5) {
        candleSubsystem.setColor(LEDConstants.green);
      } else if (Math.abs(rotate) < 12) {
        candleSubsystem.setColor(LEDConstants.yellow);
      } else {
        candleSubsystem.setColor(LEDConstants.red);
      }
    } else {
      candleSubsystem.setColor(LEDConstants.purple);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    candleSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limelight == VisionConstants.noteLimelite) {
      return true;
    } else {
      return false;
    }
  }
}
