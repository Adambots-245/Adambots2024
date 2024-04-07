package com.adambots.commands.driveCommands;

import com.adambots.Robot;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionDriveToWaypointCommand extends Command {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Gyro gyro;
  private Pose2d waypoint;
  private double finishedInc;
  private double abortInc;

  private double xPos;
  private double yPos;

  private String limelight = VisionConstants.aprilLimelite; //Can quicky change which limelight is referenced

  public VisionDriveToWaypointCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro, Pose2d waypoint) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyro = gyro;
    this.waypoint = waypoint;

    xController = new PIDController(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController);
    yController = new PIDController(AutoConstants.kPTranslationController, 0, AutoConstants.kDTranslationController);

    thetaController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kPThetaController);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(waypoint.getX());
    yController.setSetpoint(waypoint.getY());
    
    if (Robot.isOnRedAlliance()) {
      thetaController.setSetpoint(-waypoint.getRotation().getRadians());
    } else {
      thetaController.setSetpoint(waypoint.getRotation().getRadians());
    }

    finishedInc = 0;
    xPos = 0;
    yPos = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (VisionHelpers.isDetected(limelight)) {
      Pose2d visionPose = VisionHelpers.getAprilTagBotPose2dBlue(limelight); //TODO: Check pose vetting on red side
      if (visionPose.getY() > 1 && getContinuousAngleError(visionPose.getRotation().getRadians(), gyro.getContinuousYawRad()) < Math.toRadians(20)) {
        xPos = visionPose.getX();
        yPos = visionPose.getY();
        abortInc = 0;
      }
    } else {
      abortInc++;
      System.out.println(this.getName() + " | No Apriltags detected");
    }
    if (xPos != 0 && yPos != 0) {
      double x = xController.calculate(drivetrainSubsystem.getPose().getX());
      double y = yController.calculate(drivetrainSubsystem.getPose().getY());

      double theta = thetaController.calculate(gyro.getContinuousYawRad());

      x = MathUtil.clamp(x, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);
      y = MathUtil.clamp(y, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);

      if (Robot.isOnRedAlliance()) {
        drivetrainSubsystem.drive(x, -y, theta, true);
      } else {
        drivetrainSubsystem.drive(x, y, theta, true);
      }

      if (getDist(drivetrainSubsystem.getPose(), waypoint) < 0.1 && Math.abs(thetaController.getPositionError()) < Math.toRadians(5)) {
        finishedInc++;
      } else if (finishedInc > 0) {
        finishedInc--;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (abortInc > 25) {
      System.out.println(this.getName() + " | ABORTED - UNRELIABLE APRILTAG DETECTION");
      return true;
    }
    return finishedInc > 7;
  }

  public double getDist (Pose2d pos1, Pose2d pos2) {
    double x = pos1.getX() - pos2.getX();
    double y = pos1.getY() - pos2.getY();
        
    return Math.hypot(x, y);
  }

  public double getContinuousAngleError (double setpoint, double measurement) {
    return MathUtil.inputModulus(setpoint - measurement, -Math.PI, Math.PI);
  }
}
