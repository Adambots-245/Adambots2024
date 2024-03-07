package com.adambots.commands.driveCommands;

import com.adambots.Robot;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngleCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  // private CANdleSubsystem caNdleSubsystem;
  private BaseGyro gyro;
  private final PIDController angleTurningPIDController = new PIDController(VisionConstants.kPThetaController, 0,
      VisionConstants.kDThetaController);
  private double drive_output;
  private double targetAngleRad;

  public RotateToAngleCommand(DrivetrainSubsystem driveTrainSubsystem, double targetAngleDeg, BaseGyro gyro) {
    addRequirements(driveTrainSubsystem);

    angleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetAngleRad = Math.toRadians(targetAngleDeg);
    this.gyro = gyro;
  }

  @Override
  public void initialize() {
    if (Robot.isOnRedAlliance()) {
      targetAngleRad = -targetAngleRad;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculates the drive rotation
    drive_output = angleTurningPIDController.calculate(gyro.getContinuousYawRad(), targetAngleRad);

    // Moves left or right depending on the angle
    driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
        Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, drive_output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}