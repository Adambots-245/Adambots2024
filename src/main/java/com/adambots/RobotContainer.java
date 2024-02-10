
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Gamepad.Buttons;
import com.adambots.commands.ChangeArmStateCommand;
import com.adambots.commands.FeedShooterCommand;
import com.adambots.commands.RotateShoulderCommand;
import com.adambots.commands.RotateWristCommand;
import com.adambots.commands.RunIntakeCommand;
import com.adambots.commands.RunShooterCommand;
import com.adambots.commands.autonCommands.AdjustNote;
import com.adambots.commands.autonCommands.FireCommand;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.RobotMap;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.utils.Dash;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.GamepadConstants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
 private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.swerveModules, RobotMap.GyroSensor);
 private final ArmSubsystem armSubsystem = new ArmSubsystem(RobotMap.shoulderMotor, RobotMap.wristMotor, RobotMap.shoulderEncoder, RobotMap.wristEncoder, RobotMap.shoulderLowerLimit, RobotMap.wristLowerLimit);
 private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(RobotMap.shooterWheel);
 private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.groundIntakeMotor, RobotMap.firstPieceInRobotEye, RobotMap.secondPieceInRobotEye);

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
    //Debugging and Testing
    // Buttons.JoystickButton4.onTrue(armCommands.humanStationConeCommand());
    // Buttons.JoystickButton11.onTrue(Commands.deadline(new WaitCommand(1.5), autonCommands.driveTillBumpedCommand()));
    // Buttons.JoystickButton16.onTrue(new TurnToGamePieceCommand(drivetrainSubsystem, RobotMap.lidar, Direction.RIGHT));
    // Buttons.JoystickButton16.onTrue(autonCommands.autoInitAndScoreCone());

    // Buttons.JoystickButton16.onTrue(new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem).andThen(new HockeyStopCommand(drivetrainSubsystem)));
    // Buttons.JoystickButton16.onTrue(new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor, grabbyLifterSubsystem));

    // Buttons.primaryRB.whileTrue(new RunIntakeCommand(intakeSubsystem, 0.3));
    Buttons.primaryRB.whileTrue(new AdjustNote(intakeSubsystem));


    Buttons.JoystickButton1.onTrue(new InstantCommand(() -> RobotMap.GyroSensor.reset()));

    Buttons.primaryLB.onTrue(new FireCommand(shooterSubsystem, intakeSubsystem));

    //Arm State Buttons
    Buttons.primaryAButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.floorState));
    Buttons.primaryBButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.ampState));
    Buttons.primaryXButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.speakerState));
    Buttons.primaryYButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.humanState));
    Buttons.primaryStartButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.defaultState));
    Buttons.primaryBackButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.trapState));
    Buttons.primaryDPadN.onTrue(new RotateShoulderCommand(armSubsystem, 3));
    Buttons.primaryDPadS.onTrue(new RotateShoulderCommand(armSubsystem, -3));
    Buttons.primaryDPadW.onTrue(new RotateWristCommand(armSubsystem, -3));
    Buttons.primaryDPadE.onTrue(new RotateWristCommand(armSubsystem, 3));
    
  }


  private void registerNamedCommands() {
    NamedCommands.registerCommand("TestCommand1", new PrintCommand("Test1!"));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("Field", Constants.DriveConstants.field);

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("yaw", () -> RobotMap.GyroSensor.getAngle());
    Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());
    Dash.add("IntakeSpeed", () -> intakeSubsystem.getIntakeSpeed());

    Dash.add("getIntake", () -> intakeSubsystem.getIntake());



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
                Buttons.forwardSupplier.getAsDouble()*1.65,
                Buttons.sidewaysSupplier.getAsDouble()*1.65,
                Buttons.rotateSupplier.getAsDouble()*1.43,
                true),
            drivetrainSubsystem));
    intakeSubsystem.setDefaultCommand(
      new RunCommand(
      
        () -> intakeSubsystem.setGroundIntakeMotorSpeed(Buttons.deaden(Buttons.primaryJoystick.getRightY(),GamepadConstants.kDeadZone) * 0.2), 
        intakeSubsystem)
    );
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