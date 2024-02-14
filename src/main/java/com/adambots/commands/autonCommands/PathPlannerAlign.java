package com.adambots.commands.autonCommands;
import java.util.List;

import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
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
  private Pose2d targetPos;
  private static PathPlannerPath path;

  public PathPlannerAlign(DrivetrainSubsystem driveTrainSubsystem, Pose2d targetPose) {
    addRequirements(driveTrainSubsystem);
    // Dash.add("isFInsihed", () -> AutoBuilder.followPath(path).isFinished());
    this.targetPos = targetPose;
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = driveTrainSubsystem.getPose();
      Pose2d currentAprilPose = VisionHelpers.getAprilTagPose2d();
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
      Pose2d endPos;
      endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(currentAprilPose.getTranslation().getX()-targetPos.getTranslation().getX(), currentAprilPose.getTranslation().getY() - targetPos.getTranslation().getY())), new Rotation2d());

      // Pose2d endPos = new Pose2d(new Translation2d(6.7, 1.49), new Rotation2d(0));

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          5, 5, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, new Rotation2d(Math.toRadians(270)))
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;
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
      driveTrainSubsystem.stop();
  }

  public static PathPlannerPath getPath(){
    return path;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // return AutoBuilder.followPath(path).isFinished();
    return true;
  }
}
