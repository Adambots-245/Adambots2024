package com.adambots.commands.driveCommands;
import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AngleRotateCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem caNdleSubsystem;
  private BaseGyro gyro;
  private final PIDController angleTurningPIDController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
  private final double filterSens = 0.1;
  private double oldRotate;
  private double newRotate;
  private double drive_output;
  private boolean continuous;
  private String limelight;

  public AngleRotateCommand(DrivetrainSubsystem driveTrainSubsystem, CANdleSubsystem caNdleSubsystem, BaseGyro gyro) {
    addRequirements(driveTrainSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.caNdleSubsystem = caNdleSubsystem;
    this.gyro = gyro;
  }

  @Override
  public void initialize() {
    oldRotate = 90 - gyro.getContinuousYawDeg();
    drive_output = 0;
    newRotate = 90 - gyro.getContinuousYawDeg();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Filter to stop large changes in data while runing, to make the PID work better
    newRotate = filterSens*gyro.getContinuousYawDeg() + (1-filterSens)*oldRotate;
    oldRotate = newRotate;

    // Calculates the drive rotation
    drive_output = angleTurningPIDController.calculate((double)(Math.round((Math.abs(Math.toRadians(newRotate)) / 100d))) / 100d, 0);

    //Moves left or right depending on the angle
    if (newRotate > 0){
      driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , drive_output, true);
    } else {
      driveTrainSubsystem.drive(-Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond, -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond , -drive_output, true);
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If the robot is aligned for some time, or the robot is not detecting a piece the command ends
    return false;
  }
}
