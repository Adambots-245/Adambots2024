// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.commands.FeedShooterCommand;
import com.adambots.commands.RunShooterCommand;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    addCommands(new InstantCommand(() -> shooterSubsystem.setWheelSpeed(1)), new WaitCommand(2), new InstantCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(-0.5)), new WaitCommand(1), new InstantCommand(() -> intakeSubsystem.setGroundIntakeMotorSpeed(0)), new InstantCommand(() -> shooterSubsystem.setWheelSpeed(0)));
  }
}
