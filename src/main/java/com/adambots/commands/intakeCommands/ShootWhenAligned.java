// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.intakeCommands;

import com.adambots.Constants.VisionConstants;
import com.adambots.Constants.ArmConstants.StateName;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.LEDConstants;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.vision.VisionHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootWhenAligned extends Command {
  /** Creates a new FeedShooterCommand. */
  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private int inc;
  private boolean increment;
  private DrivetrainSubsystem driveTrainSubsystem;
  private CANdleSubsystem candleSubsystem;
  private PIDController turningPIDController = new PIDController(VisionConstants.kPThetaController, 0, VisionConstants.kDThetaController);
  private double rotation_output;


  public ShootWhenAligned(DrivetrainSubsystem drivetrainSubsystem, CANdleSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
    ShooterSubsystem shooterSubsystem) {
    addRequirements(intakeSubsystem, drivetrainSubsystem);

    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.driveTrainSubsystem = drivetrainSubsystem;
    this.candleSubsystem = ledSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
    increment = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.isAtTargetSpeed() && (armSubsystem.getCurrentStateName() == StateName.SPEAKER || armSubsystem.getCurrentStateName() == StateName.CLOSE_FLOOR_SHOOT) 
    && armSubsystem.isAtTargetState() && VisionHelpers.isAligned(VisionConstants.aprilLimelite, 5)/*  && VisionHelpers.getAprilHorizDist() < 3*/) {
      intakeSubsystem.setMotorSpeed(IntakeConstants.shootSpeed);
      increment = true;
    }
    if (increment) {
      inc++;
    }

    double rotate = VisionHelpers.getHorizAngle(VisionConstants.aprilLimelite);

    // Calculates the drive rotation
    if (VisionHelpers.getAprilTagID() == 4 || VisionHelpers.getAprilTagID() == 7) {
      rotation_output = turningPIDController.calculate(Math.toRadians(rotate), 0);
    } else {
      rotation_output = 0;
    }

    // Checks to see if we have an object detected
    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
          Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond, rotation_output, true);
    } else {
      driveTrainSubsystem.drive(Buttons.forwardSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
          Buttons.sidewaysSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
          Buttons.rotateSupplier.getAsDouble() * DriveConstants.kTeleopRotationalSpeed, true);
    }

    if (VisionHelpers.isDetected(VisionConstants.aprilLimelite)) {
      if (Math.abs(rotate) < 5) {
        candleSubsystem.setColor(LEDConstants.green);
      } else if (Math.abs(rotate) < 12) {
        candleSubsystem.setColor(LEDConstants.yellow);
      } else {
        candleSubsystem.setColor(LEDConstants.red);
      }
    } else {
      candleSubsystem.setColor(LEDConstants.purple);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotorSpeed(0);
    shooterSubsystem.setTargetWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc > 15;
  }
}
