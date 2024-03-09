// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.hangCommands;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.HangConstants;
import com.adambots.sensors.BaseGyro;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.HangSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class HangLevelCommand extends Command {
  private HangSubsystem hangSubsystem;
  private ArmSubsystem armSubsystem;
  private BaseGyro gyro;
  private double speed = -0.65;

  public HangLevelCommand(HangSubsystem hangSubsystem, ArmSubsystem armSubsystem, BaseGyro gyro) {
    addRequirements(hangSubsystem, armSubsystem);
    this.armSubsystem = armSubsystem;
    this.hangSubsystem = hangSubsystem;
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setCurrentState(ArmConstants.hangState);
    hangSubsystem.setSolenoids(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroFactor = gyro.getRoll()*0.02; 

    hangSubsystem.setLeftMotorSpeed(MathUtil.clamp(speed+gyroFactor, -1, 0));
    hangSubsystem.setRightMotorSpeed(MathUtil.clamp(speed-gyroFactor, -1, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangSubsystem.setLeftMotorSpeed(0);
    hangSubsystem.setRightMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Finish when one of the motors is fully retracted and the robot is level
    return (hangSubsystem.getLeftMotorPosition() <= HangConstants.lowExtension || hangSubsystem.getRightMotorPosition() <= HangConstants.lowExtension) && Math.abs(gyro.getRoll()) < 5;
  }
}
