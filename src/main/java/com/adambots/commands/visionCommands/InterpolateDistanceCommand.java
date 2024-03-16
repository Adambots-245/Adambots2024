package com.adambots.commands.visionCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ArmConstants.StateName;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.ShooterPreset;
import com.adambots.utils.VisionLookUpTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class InterpolateDistanceCommand extends Command {
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private DrivetrainSubsystem drivetrainSubsystem;

  public InterpolateDistanceCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void initialize() {

    shooterSubsystem.setTargetWheelSpeed(80);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = drivetrainSubsystem.getPose(); //Get odometry data from drivetrain
    Translation2d currentTranslation = currentPose.getTranslation();
   
      double targetDistance = Math.hypot(VisionConstants.blueTargetPoint.getY()-currentTranslation.getY(), VisionConstants.blueTargetPoint.getX()-currentTranslation.getX());

      ShooterPreset preset = VisionLookUpTable.getShooterPreset(targetDistance);

      if (Math.abs(armSubsystem.getCurrentState().getWristAngle() - preset.getWristAngle()) > 0.5){
        State state = new State(preset.getWristAngle(), 125, StateName.CUSTOM);
        armSubsystem.setCurrentState(state);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooterSubsystem.setTargetWheelSpeed(0);
      armSubsystem.setCurrentState(ArmConstants.defaultState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
