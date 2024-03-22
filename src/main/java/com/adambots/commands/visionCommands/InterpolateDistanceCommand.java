package com.adambots.commands.visionCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ArmConstants.StateName;
import com.adambots.Constants.VisionConstants;
import com.adambots.Robot;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.ShooterPreset;
import com.adambots.utils.VisionLookUpTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class InterpolateDistanceCommand extends Command {
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private DrivetrainSubsystem drivetrainSubsystem;
  private IntakeSubsystem intakeSubsystem;
  // ShooterPreset preset = VisionLookUpTable.getShooterPreset(1);

  public InterpolateDistanceCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Dash.add("Preset Angle", () -> preset.getWristAngle());
  }

  @Override
  public void initialize() {
    armSubsystem.resetDebounce();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intakeSubsystem.getLockOut()) {
      shooterSubsystem.setTargetWheelSpeed(90);
    }

    Pose2d currentPose = drivetrainSubsystem.getPose(); //Get odometry data from drivetrain
    Translation2d currentTranslation = currentPose.getTranslation();
   
    Translation2d targetPose = VisionConstants.blueTargetPoint;
    if (Robot.isOnRedAlliance()) {
      targetPose = VisionConstants.redTargetPoint;
    }

    double targetDistance = Math.hypot(targetPose.getY()-currentTranslation.getY(), targetPose.getX()-currentTranslation.getX());

    ShooterPreset preset = VisionLookUpTable.getShooterPreset(targetDistance);
    SmartDashboard.putNumber("Preset Angle", preset.getWristAngle());

    // if (Math.abs(armSubsystem.getCurrentState().getWristAngle() - preset.getWristAngle()) > 0.5){
      State state = new State(preset.getWristAngle(), 125, StateName.CUSTOM);
      armSubsystem.setCurrentState(state);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!DriverStation.isAutonomous()) {
      shooterSubsystem.setTargetWheelSpeed(0);
      armSubsystem.setCurrentState(ArmConstants.defaultState);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous()){
      if (armSubsystem.getCurrentStateName() == ArmConstants.StateName.CUSTOM){
        return armSubsystem.isAtTargetState();
      }
    }
    return false;
  }
}
