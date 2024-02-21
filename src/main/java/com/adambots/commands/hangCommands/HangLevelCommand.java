// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.hangCommands;

import com.adambots.sensors.Gyro;
import com.adambots.subsystems.HangSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class HangLevelCommand extends Command {
  private HangSubsystem hangSubsystem;
  private Gyro gyro;
  private double speed = 0.25;

  public HangLevelCommand(HangSubsystem hangSubsystem, Gyro gyro) {
    addRequirements(hangSubsystem);

    this.hangSubsystem = hangSubsystem;
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hangSubsystem.setSolenoids(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroFactor = gyro.getPitch()*0.07; //TODO: determing if this should be pitch or roll

    hangSubsystem.setLeftMotorSpeed(speed+gyroFactor);
    hangSubsystem.setRightMotorSpeed(speed-gyroFactor);
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
    return (hangSubsystem.getLeftMotorPosition() <= 0 || hangSubsystem.getRightMotorPosition() <= 0) && Math.abs(gyro.getRoll()) < 5;
  }
}
