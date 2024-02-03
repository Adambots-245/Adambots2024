
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Constants.DriveConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.autonCommands.autonCommandGrounds.PickupCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.gyro);

  //Creates a SmartDashboard element to allow drivers to select differnt autons
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setupDefaultCommands();

    // Register Commands for use in PathPlanner
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // configure the dashboard
    setupDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   */
  
  private void configureButtonBindings() {
    Buttons.JoystickButton1.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw()));
    // Buttons.primaryAButton.whileTrue(new AlignNoteCommand(drivetrainSubsystem));
    // Buttons.primaryBButton.whileTrue(new AlignNoteDistanceCommand(drivetrainSubsystem));
    Buttons.primaryYButton.onTrue(new PickupCommand(drivetrainSubsystem));

    //Debugging and Testing
    Buttons.JoystickButton4.onTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d())));
    // Buttons.JoystickButton11.onTrue(Commands.deadline(new WaitCommand(1.5), autonCommands.driveTillBumpedCommand()));
    // Buttons.JoystickButton16.onTrue(new TurnToGamePieceCommand(drivetrainSubsystem, RobotMap.lidar, Direction.RIGHT));
    // Buttons.JoystickButton16.onTrue(autonCommands.autoInitAndScoreCone());

    // Buttons.JoystickButton16.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton16.onTrue(new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("TestCommand1", new PrintCommand("Test1!"));
    NamedCommands.registerCommand("ResetGyro", new InstantCommand(() -> RobotMap.gyro.resetYaw()));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);

   // Dash.add("Field", Constants.field);

  //  Dash.add("xPID", () -> AlignNoteCommand);

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());

    Dash.add("getClassName", VisionHelpers.getClassName());
    Dash.add("getXLocation", () ->VisionHelpers.getXLocation());
    Dash.add("getYLocation", () ->VisionHelpers.getYLocation());
    Dash.add("getDistanceToObject", () ->VisionHelpers.getDistanceToObject());
    Dash.add("isDetected", () ->VisionHelpers.isDetected());
    Dash.add("getVertAngle", () ->VisionHelpers.getVertAngle());
    Dash.add("getHorizAngle", () ->VisionHelpers.getHorizAngle());
    Dash.add("isAligned", () ->VisionHelpers.isAligned());
    Dash.add("isDistanceAligned", () ->VisionHelpers.isDistanceAligned());


    // Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    // Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());

    // Dash.add("LIDAR Dist", () -> RobotMap.lidar.getInches());

    // // Dash.add("Vision X:" , () -> VisionHelpers.getAprilTagPose2d().getX());
    // // Dash.add("Vision Y:" , () -> VisionHelpers.getAprilTagPose2d().getY());
    // // Dash.add("Vision Index:" , () -> VisionHelpers.getDetectedResult());

    // Dash.add("isDetectingPieces", () -> VisionHelpers.isDetectingPieces("cube"));
    // Dash.add("pieceX", () -> VisionHelpers.getPieceX("cube"));
    // Dash.add("pieceY", () -> VisionHelpers.getPieceY("cube"));
    // Dash.add("DistanceToObject", () -> VisionHelpers.getDistanceToObject());
    // Dash.add("tx", () -> VisionHelpers.getTX());
    // Dash.add("Aligned", () -> VisionHelpers.isAligned());
  }

  private void setupDefaultCommands() {
    drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                -Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                -Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                -Buttons.rotateSupplier.getAsDouble()*DriveConstants.kTeleopRotationalSpeed,
                true),
            drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
//Blahaj_Counter: 2
