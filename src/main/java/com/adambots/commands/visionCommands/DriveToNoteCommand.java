package com.adambots.commands.visionCommands;
import org.opencv.core.Mat;

import com.adambots.Robot;
import com.adambots.RobotMap;
import com.adambots.Constants.LEDConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNoteCommand extends Command {
  private DrivetrainSubsystem driveTrainSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private CANdleSubsystem ledSubsystem;
  private final PIDController pidController = new PIDController(VisionConstants.kPTranslateController, 0, VisionConstants.kDTranslateController);
  private final PIDController rotatePidController = new PIDController(0.1, 0, 0.00001);
  private double drive_output;
  private double speed;
  private double debounce;
  public DriveToNoteCommand(DrivetrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, CANdleSubsystem ledSubsystem, double speed) {
    addRequirements(driveTrainSubsystem);

    rotatePidController.enableContinuousInput(-Math.PI, Math.PI);

    this.intakeSubsystem = intakeSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(LEDConstants.red);
    pidController.reset();
    if (DriverStation.isAutonomous()) {
      rotatePidController.setSetpoint(0);
    } else {
      rotatePidController.setSetpoint(RobotMap.gyro.getContinuousYawRad());
    }
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!VisionHelpers.isDetected(VisionConstants.noteLimelite)){
      debounce++;
    } else {
      debounce = 0;
    }

    drive_output = pidController.calculate(VisionHelpers.getHorizAngle(VisionConstants.noteLimelite), 0);
    double rotate_output = rotatePidController.calculate(RobotMap.gyro.getContinuousYawRad());
    driveTrainSubsystem.drive(speed, drive_output, rotate_output, false);

    double rotate = VisionHelpers.getHorizAngle(VisionConstants.noteLimelite);

    if (VisionHelpers.isDetected(VisionConstants.noteLimelite)) {
      if (Math.abs(rotate) < 5) {
        ledSubsystem.setColor(LEDConstants.green);
      } else if (Math.abs(rotate) < 12) {
        ledSubsystem.setColor(LEDConstants.yellow);
      } else {
        ledSubsystem.setColor(LEDConstants.red);
      }
    } else {
      ledSubsystem.setColor(LEDConstants.purple);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrainSubsystem.stop();
      System.out.println("DriveToNoteCommand DONE" + interrupted);
      ledSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (DriverStation.isAutonomous()){
          return debounce > 50 || intakeSubsystem.isFirstPieceInRobot();
      } else {
        return intakeSubsystem.isFirstPieceInRobot();
      }
  }
}
