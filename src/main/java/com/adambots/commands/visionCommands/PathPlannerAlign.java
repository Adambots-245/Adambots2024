package com.adambots.commands.visionCommands;
import java.util.List;

import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerAlign extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private static PathPlannerPath path;
  private Pose2d targetPos;

  public PathPlannerAlign(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    Dash.add("April Tag ID", () -> VisionHelpers.getAprilTagID());
  }

  @Override
  public void initialize() {
    // Gets the current position of the robot
    Pose2d currentPos = driveTrainSubsystem.getPose();
    // Gets the robots pos calculated from the april tag
    Pose2d currentAprilPos = VisionHelpers.getAprilTagPose2d();
    Pose2d startPos = new Pose2d(currentPos.getTranslation(), currentPos.getRotation());
    // Adds distance from the current robot pose, to align to the desired target pos
    targetPos = VisionConstants.aprilTagRedPose2d;
    Pose2d endPos = new Pose2d(currentPos.getTranslation().getX(), currentAprilPos.getTranslation().getY(), new Rotation2d());
    if (VisionHelpers.getAprilTagID() == 4){
      targetPos = VisionConstants.aprilTagRedPose2d;
      endPos = new Pose2d(currentPos.getTranslation().plus(new Translation2d(-(currentAprilPos.getTranslation().getX()-targetPos.getTranslation().getX()), -(currentAprilPos.getTranslation().getY() - targetPos.getTranslation().getY()))), new Rotation2d());
    }
    // Creates a path to follow
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    path = new PathPlannerPath(
      bezierPoints, 
      new PathConstraints(
        5, 5, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ),  
      new GoalEndState(0.0, new Rotation2d(Math.toRadians(0)))
    );

    // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    path.preventFlipping = true;

    // Only moves toward the path if there is an april tag detected
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)){
      AutoBuilder.followPath(path).schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return AutoBuilder.followPath(path).isFinished();
    return (Math.abs(VisionHelpers.getAprilTagPose2d().getTranslation().getX() - targetPos.getTranslation().getX()) < 1) && (Math.abs(VisionHelpers.getAprilTagPose2d().getTranslation().getY() - VisionConstants.aprilTagRedPose2d.getTranslation().getY()) < 1);
  }
}
