package com.adambots.commands.visionCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.ArmConstants.State;
import com.adambots.Constants.ArmConstants.StateName;
import com.adambots.Constants.ShooterConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.ShooterPreset;
import com.adambots.utils.VisionLookUpTable;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.wpilibj2.command.Command;

public class InterpolateDistanceCommand extends Command {
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;

  private double oldDistance = 0;
  private double distance = 0;

  private final double sens = 0.095;

  public InterpolateDistanceCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      oldDistance = VisionHelpers.getAprilHorizDist();
      distance = VisionHelpers.getAprilHorizDist();
    } else {
      oldDistance = 2; 
      distance = 2;
    }
          shooterSubsystem.setTargetWheelSpeed(80);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (VisionHelpers.isDetected(VisionConstants.aprilLimelite) && armSubsystem.getCurrentState().getStateName().toString() != "CUSTOM") {
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      distance = (1-sens)*oldDistance + sens*VisionHelpers.getAprilHorizDist();
      oldDistance = distance;

      ShooterPreset preset = VisionLookUpTable.getShooterPreset(distance);

      // State state = new State(317, 125, StateName.CUSTOM, 25, 25);
      if (Math.abs(armSubsystem.getCurrentState().getWristAngle() - preset.getWristAngle()) > 0.5){
          State state = new State(preset.getWristAngle(), 125, StateName.CUSTOM);
          armSubsystem.setCurrentState(state);
      }
      // armSubsystem.setCurrentState(ArmConstants.floorState);


      System.out.println("Updating State, Distance: " + distance);
      System.out.println("getWristAngle" + preset.getWristAngle());
      System.out.println("getShoulderAngle " + preset.getArmAngle());

    } else {
      System.out.println("No Tag Found");
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
