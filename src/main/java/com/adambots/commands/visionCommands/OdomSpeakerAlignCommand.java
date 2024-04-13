package com.adambots.commands.visionCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Robot;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class OdomSpeakerAlignCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem candleSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ArmSubsystem armSubsystem;
  // private double activateDelay;
  private String limelight;
  private PIDController turningPIDController = new PIDController(VisionConstants.kPOdomThetaController, 0, VisionConstants.kDOdomThetaController);

  public OdomSpeakerAlignCommand(DrivetrainSubsystem driveTrainSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, CANdleSubsystem ledSubsystem, String limelight) {
    addRequirements(driveTrainSubsystem);


    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
    this.armSubsystem = armSubsystem;
    this.candleSubsystem = ledSubsystem;
  }

  @Override
  public void initialize() {
    candleSubsystem.setColor(LEDConstants.yellow);
    driveTrainSubsystem.setArmLimelightFlag(true);
    // activateDelay = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (VisionHelpers.isDetected(VisionConstants.aprilLimelite) || activateDelay > 17) {
      Pose2d currentPose = driveTrainSubsystem.getPose(); //Get odometry data from drivetrain
      double currentRotation = currentPose.getRotation().getRadians();
      Translation2d currentTranslation = currentPose.getTranslation();

      Translation2d targetPose = VisionConstants.blueTargetPoint;
      if (Robot.isOnRedAlliance()) {
        targetPose = VisionConstants.redTargetPoint;
      }
      double targetRotation;
      //Calculate angle to speaker  
      // if (limelight == VisionConstants.aprilLimelite){
      //   targetRotation = Math.atan2(targetPose.getY()-currentTranslation.getY(), targetPose.getX()-currentTranslation.getX()) + Math.PI;
      // } else{
        targetRotation = Math.atan2(targetPose.getY()-currentTranslation.getY(), targetPose.getX()-currentTranslation.getX());
      // }

      //Calculate and apply the nessecary rotation
      double rotation_output = turningPIDController.calculate(currentRotation, targetRotation);
      // if (limelight == VisionConstants.aprilLimelite){
      //   rotation_output = turningPIDController.calculate(currentRotation + Math.PI, targetRotation);
      // } 
      //   rotation_output = turningPIDController.calculate(currentRotation, targetRotation);
      // }
      driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
      Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, rotation_output, true);

      //Light up LEDs depending on our alignment
      double absErrorDeg = Math.abs(Math.toDegrees(turningPIDController.getPositionError()));
      if (DriverStation.isAutonomous()){
        absErrorDeg = Math.abs(Math.toDegrees(turningPIDController.getPositionError()));
      } 
      //   rotation_output = turningPIDController.calculate(currentRotation, targetRotation);
      // }
      if(armSubsystem.getCurrentStateName() == ArmConstants.StateName.CUSTOM){
        if (armSubsystem.isAtTargetStateTele() && absErrorDeg < 5 && shooterSubsystem.getShooterVelocity() >= 88){
          candleSubsystem.setColor(LEDConstants.purple);
        }
      } else if (absErrorDeg < 5) {
        candleSubsystem.setColor(LEDConstants.green);
      } else if (absErrorDeg < 12) {
        candleSubsystem.setColor(LEDConstants.yellow);
      } else {
        candleSubsystem.setColor(LEDConstants.red);
      }
    // // } else {
    //   activateDelay++;
    //   driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
    //   Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, 
    //   Buttons.rotateSupplier.getAsDouble() * DriveConstants.kTeleopRotationalSpeed, true);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    driveTrainSubsystem.setArmLimelightFlag(false);
    candleSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous()){
      System.out.print("ODEN ALIGNED");
      return Math.abs(Math.toDegrees(turningPIDController.getPositionError())) < 2;
    }
    return false;
  }
}
