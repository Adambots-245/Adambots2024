package com.adambots.commands.driveCommands;

import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Constants;
import com.adambots.Robot;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionDriveToWaypointCommand extends Command {
  private DrivetrainSubsystem drivetrainSubsystem;
  private PIDController xController = new PIDController(AutoConstants.kPWaypointTranslation, 0, AutoConstants.kDWaypointTranslation);
  private PIDController yController = new PIDController(AutoConstants.kPWaypointTranslation, 0, AutoConstants.kDWaypointTranslation);
  private PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
  private BaseGyro gyro;
  private Pose2d waypoint;

  private double finishedInc;
  private double abortInc;

  private double xPos;
  private double yPos;
  private double xPosOld;
  private double yPosOld;

  private final double posSensitivity = 0.7;
  private final double abortThreshold = 25;
  private final String limelight = VisionConstants.aprilLimelite; //Can quicky change which limelight is referenced

  public VisionDriveToWaypointCommand(DrivetrainSubsystem drivetrainSubsystem, BaseGyro gyro, Pose2d waypoint) {
    addRequirements(drivetrainSubsystem);

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyro = gyro;
    this.waypoint = waypoint;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.isOnRedAlliance()) {
      xController.setSetpoint(waypoint.getX() + VisionConstants.kFieldWidth);
    } else {
      xController.setSetpoint(waypoint.getX());
    }
    yController.setSetpoint(waypoint.getY());

    finishedInc = 0;
    abortInc = 0;
    xPos = 0;
    yPos = 0;
    xPosOld = 0;
    yPosOld = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    yPos = drivetrainSubsystem.getPose().getY();
    xPosOld = xPos;
    yPosOld = yPos;
      
    if (xPos != 0 && yPos != 0) {
      double xDrive = xController.calculate(xPos);
      double yDrive = yController.calculate(yPos);
      
      if (Math.abs(waypoint.getX()-xPos) > 1.25 || Math.abs(waypoint.getY()-yPos) > 0.7){
        if (Robot.isOnRedAlliance()) { //If the robot is further than 1.25 meters in x, rotate to face the apriltags instead of the waypoint to maintain them in FOV
          thetaController.setSetpoint(Math.atan2(VisionConstants.aprilTagPos.getY()-yPos, VisionConstants.aprilTagPos.getX()+VisionConstants.kFieldWidth-xPos));
        } else {
          thetaController.setSetpoint(Math.atan2(VisionConstants.aprilTagPos.getY()-yPos, VisionConstants.aprilTagPos.getX()-xPos));
        }
      } else {
        if (Robot.isOnRedAlliance()) {
          thetaController.setSetpoint(-waypoint.getRotation().getRadians());
        } else {
          thetaController.setSetpoint(waypoint.getRotation().getRadians());
        }
      }
      double thetaDrive = thetaController.calculate(gyro.getContinuousYawRad());

      if (getDist(waypoint, drivetrainSubsystem.getPose()) > 1.2) {
        xDrive = MathUtil.clamp(xDrive, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);
        yDrive = MathUtil.clamp(yDrive, -AutoConstants.kMaxWaypointTranslateSpeed, AutoConstants.kMaxWaypointTranslateSpeed);
      } else {
        xDrive = MathUtil.clamp(xDrive, -AutoConstants.kMinWaypointTranslateSpeed, AutoConstants.kMinWaypointTranslateSpeed);
        yDrive = MathUtil.clamp(yDrive, -AutoConstants.kMinWaypointTranslateSpeed, AutoConstants.kMinWaypointTranslateSpeed);
      }
      

      if (Robot.isOnRedAlliance()) {
        drivetrainSubsystem.drive(xDrive, -yDrive, thetaDrive, true);
      } else {
        drivetrainSubsystem.drive(xDrive, yDrive, thetaDrive, true);
      }

      if (Math.abs(waypoint.getX()-xPos) < 0.1 && Math.abs(waypoint.getY()-yPos) < 0.1 && Math.abs(thetaController.getPositionError()) < Math.toRadians(5)) {
        finishedInc++;
      } else if (finishedInc > 0) {
        finishedInc--;
      }
    }
    // Constants.debugField.setRobotPose(new Pose2d(xPos, yPos, new Rotation2d(drivetrainSubsystem.getPose().getRotation().getRadians())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (abortInc <= abortThreshold) {
    //   drivetrainSubsystem.resetOdometryXY(waypoint.getTranslation());
    // }
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (abortInc > abortThreshold) {
      System.out.println(this.getName() + " | ABORTED - UNRELIABLE APRILTAG DETECTION");
      return true;
    }
    return finishedInc > 25;
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
