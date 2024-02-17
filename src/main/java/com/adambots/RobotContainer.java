package com.adambots;

import com.adambots.Constants.*;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.PathPlannerAlign;
import com.adambots.commands.visionCommands.AlignRotateCommand;
import com.adambots.subsystems.*;
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
// import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private final ArmSubsystem armSubsystem = new ArmSubsystem(RobotMap.shoulderMotor, RobotMap.wristMotor, RobotMap.shoulderEncoder, RobotMap.wristEncoder);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(RobotMap.shooterWheel);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.groundIntakeMotor, RobotMap.firstPieceInRobotEye, RobotMap.secondPieceInRobotEye);
  // private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.leftHangMotor, RobotMap.rightHangMotor, RobotMap.leftRelay, RobotMap.rightRelay);

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
    // Buttons.JoystickButton1.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw()));
    // Buttons.JoystickButton2.onTrue(new SequentialCommandGroup(new PathPlannerAlign(drivetrainSubsystem, VisionConstants.aprilTagPose2d), new PathPlannerAlign(drivetrainSubsystem, VisionConstants.aprilTagPose2d)));
    Buttons.JoystickButton2.onTrue(new PathPlannerAlign(drivetrainSubsystem));
    Buttons.JoystickButton13.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw()));
    
    //Debugging and Testing
    Buttons.JoystickButton4.onTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d())));
    //Buttons.JoystickButton2.onTrue(new PickupGamepieceCommand(drivetrainSubsystem));

    // Buttons.primaryDPadE.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.defaultState));
    // Buttons.primaryXButton.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.trapState));
    
    //Debugging and Testing
    // Buttons.JoystickButton2.onTrue(new PickupGamepieceCommand(drivetrainSubsystem));
    // Buttons.primaryAButton.whileTrue(new AlignNoteCommand(drivetrainSubsystem));
    // Buttons.primaryBButton.whileTrue(new AlignNoteDistanceCommand(drivetrainSubsystem));

    Buttons.JoystickButton1.whileTrue(new AdaptiveScoreCommand(armSubsystem, shooterSubsystem, intakeSubsystem));

    // Buttons.JoystickButton6.onTrue(new PickupGamepieceStrafeCommand(drivetrainSubsystem));
    // Buttons.JoystickButton7.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.aprilLimelite));
    // Buttons.JoystickButton5.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.noteLimelite));
    // Buttons.JoystickButton10.whileTrue(new AlignRotateCommand(drivetrainSubsystem, false, true, VisionConstants.noteLimelite));
    // Buttons.JoystickButton9.whileTrue(new PickupGamepieceRotateCommand(drivetrainSubsystem));
    // Buttons.JoystickButton7.onTrue(new AlignNoteBothCommand(drivetrainSubsystem));


    Buttons.primaryAButton.whileTrue(new IntakeWithAdjustCommand(armSubsystem, intakeSubsystem));
    Buttons.primaryBButton.whileTrue(new HumanStationCommand(armSubsystem, intakeSubsystem));
    Buttons.primaryXButton.whileTrue(new AmpCommand(armSubsystem));

    Buttons.primaryYButton.onTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem));
    Buttons.primaryYButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    // Buttons.primaryStartButton.onTrue(new AdjustNoteCommand(intakeSubsystem));

    Buttons.primaryDPadN.whileTrue(new RotateShoulderCommand(armSubsystem,1, true));
    Buttons.primaryDPadS.whileTrue(new RotateShoulderCommand(armSubsystem, -0.1, true));
    
    Buttons.primaryDPadE.whileTrue(new RotateWristCommand(armSubsystem, -0.5, true));
    Buttons.primaryDPadW.whileTrue(new RotateWristCommand(armSubsystem, 0.5, true));

    //Hang buttons
    // Buttons.primaryLeftStickButton.onTrue(new InstantCommand(() -> hangSubsystem.setRelay(true)));
    // Buttons.primaryRightStickButton.onTrue(new InstantCommand(() -> hangSubsystem.setRelay(false)));

    Buttons.JoystickButton7.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.aprilLimelite));
    Buttons.JoystickButton5.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.noteLimelite));
    Buttons.JoystickButton10.whileTrue(new AlignRotateCommand(drivetrainSubsystem, false, true, VisionConstants.noteLimelite));
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
    SmartDashboard.putData("AprilTagField",Constants.aprilTagfield);   
    SmartDashboard.putData("Field",Constants.field);

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    // Dash.add("yaw", () -> RobotMap.gyro.getAngle());
    // Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    // Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());
    Dash.add("IntakeSpeed", () -> intakeSubsystem.getIntakeSpeed());


    Dash.add("ClassName", VisionHelpers.getClassName(VisionConstants.noteLimelite));
    // Dash.add("XLocation", () ->VisionHelpers.getXLocation());
    // Dash.add("YLocation", () ->VisionHelpers.getYLocation());
    Dash.add("Detected", () ->VisionHelpers.isDetected(VisionConstants.noteLimelite));
    Dash.add("VertAngle", () ->VisionHelpers.getVertAngle(VisionConstants.noteLimelite));
    Dash.add("HorizAngle", () ->VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    Dash.add("Aligned", () ->VisionHelpers.isAligned(VisionConstants.noteLimelite));
    Dash.add("DistanceAligned", () ->VisionHelpers.isDistanceAligned(VisionConstants.noteLimelite));

    // Dash.add("pitch", () -> RobotMap.GyroSensor.getPitch());
    // Dash.add("roll", () -> RobotMap.GyroSensor.getRoll());

    // Dash.add("LIDAR Dist", () -> RobotMap.lidar.getInches());
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
    intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> intakeSubsystem.setGroundIntakeMotorSpeed(Buttons.deaden(Buttons.primaryJoystick.getRightY(),GamepadConstants.kDeadZone) * 0.1), 
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
//Blahaj_Counter: 3
