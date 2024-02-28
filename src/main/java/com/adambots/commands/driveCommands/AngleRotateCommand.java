package com.adambots.commands.driveCommands;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class AngleRotateCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  // private CANdleSubsystem caNdleSubsystem;
  private BaseGyro gyro;
  private final PIDController angleTurningPIDController = new PIDController(VisionConstants.kPAprilThetaController, 0, VisionConstants.kDAprilThetaController);
  private double drive_output;
  private double targetAngle;

  public AngleRotateCommand(DrivetrainSubsystem driveTrainSubsystem, double targetAngle, BaseGyro gyro) {
    addRequirements(driveTrainSubsystem);

    angleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // angleTurningPIDController.disableContinuousInput();

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetAngle = targetAngle;
    this.gyro = gyro;
  }

  @Override
  public void initialize() {
    drive_output = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = gyro.getContinuousYawDeg();

    double adjustedTargetAngle = targetAngle;
    // Calculates the drive rotation
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        adjustedTargetAngle = -targetAngle;
      };
    }
    drive_output = angleTurningPIDController.calculate(Math.toRadians(rotate), Math.toRadians(adjustedTargetAngle));

    //Moves left or right depending on the angle
    driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , drive_output, true);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If the robot is aligned for some time, or the robot is not detecting a piece the command ends
    return false;
  }
}