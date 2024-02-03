// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.commands.FeedShooterCommand;
import com.adambots.commands.RunShooterCommand;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireCommand extends SequentialCommandGroup {
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  public FireCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunShooterCommand(shooterSubsystem, 1, false), 
      new WaitCommand(2), 
      new FeedShooterCommand(intakeSubsystem, 0.5, false), 
      new WaitCommand(1), new FeedShooterCommand(intakeSubsystem, 0, false), 
      new RunShooterCommand(shooterSubsystem, 1, false));
  }
}
