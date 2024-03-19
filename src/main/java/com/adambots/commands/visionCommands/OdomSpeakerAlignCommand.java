package com.adambots.commands.visionCommands;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class OdomSpeakerAlignCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem candleSubsystem;
  private PIDController turningPIDController = new PIDController(VisionConstants.kPOdomThetaController, 0, VisionConstants.kDOdomThetaController);

  public OdomSpeakerAlignCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem ledSubsystem) {
    addRequirements(driveTrainSubsystem);

    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.candleSubsystem = ledSubsystem;
  }

  //TODO: add support for red side

  @Override
  public void initialize() {
    candleSubsystem.setColor(LEDConstants.yellow);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveTrainSubsystem.getPose(); //Get odometry data from drivetrain
    double currentRotation = currentPose.getRotation().getRadians();
    Translation2d currentTranslation = currentPose.getTranslation();
    //Calculate angle to speaker    
    double targetRotation = Math.atan2(VisionConstants.blueTargetPoint.getY()-currentTranslation.getY(), VisionConstants.blueTargetPoint.getX()-currentTranslation.getX()) + Math.PI;

    //Calculate and apply the nessecary rotation
    double rotation_output = turningPIDController.calculate(currentRotation, targetRotation);
    driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
          Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, rotation_output, true);

    //Light up LEDs depending on our alignment
    double absErrorDeg = Math.abs(Math.toDegrees(turningPIDController.getPositionError()));
    if (absErrorDeg < 5) {
      candleSubsystem.setColor(LEDConstants.green);
    } else if (absErrorDeg < 12) {
      candleSubsystem.setColor(LEDConstants.yellow);
    } else {
      candleSubsystem.setColor(LEDConstants.red);
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
    return false;
  }
}
