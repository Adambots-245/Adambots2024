package com.adambots.commands.visionCommands;
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

  private PIDController translateController = new PIDController(VisionConstants.kPTranslateController, 0, VisionConstants.kDTranslateController);
  private PIDController thetaController = new PIDController(VisionConstants.kPThetaController, 0, VisionConstants.kDThetaController);

  private double speed;
  private double debounce;


  public DriveToNoteCommand(DrivetrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, CANdleSubsystem ledSubsystem, double speed) {
    addRequirements(driveTrainSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(LEDConstants.red);
    debounce = 0;

    if (Robot.isOnRedAlliance()) {
      thetaController.setSetpoint(Math.PI);
    } else {
      thetaController.setSetpoint(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!VisionHelpers.isDetected(VisionConstants.noteLimelite)){
      debounce++;
    } else {
      debounce = 0;
    }

    double horizAngle = VisionHelpers.getHorizAngle(VisionConstants.noteLimelite);
    
    double translate_output = translateController.calculate(horizAngle, 0);
    double theta_output = thetaController.calculate(RobotMap.gyro.getContinuousYawRad());

    driveTrainSubsystem.drive(speed, translate_output, theta_output, false);

    if (VisionHelpers.isDetected(VisionConstants.noteLimelite)) {
      if (Math.abs(horizAngle) < 5) {
        ledSubsystem.setColor(LEDConstants.green);
      } else {
        ledSubsystem.setColor(LEDConstants.yellow);
      }
    } else {
      ledSubsystem.setColor(LEDConstants.purple);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();

    ledSubsystem.setColor(LEDConstants.adambotsYellow);
    ledSubsystem.setAnimation(CANdleSubsystem.AnimationTypes.Larson);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous()){
      return debounce > 50;
    }
    return intakeSubsystem.isFirstPieceInRobot();
  }
}
