package com.adambots;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.GamepadConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.AdaptiveScoreCommand;
import com.adambots.commands.armCommands.AmpCommand;
import com.adambots.commands.armCommands.HumanStationCommand;
import com.adambots.commands.armCommands.PrimeShooterCommand;
import com.adambots.commands.armCommands.RetractShooterCommand;
import com.adambots.commands.armCommands.RotateShoulderCommand;
import com.adambots.commands.armCommands.RotateWristCommand;
import com.adambots.commands.autonCommands.FloorIntakeCommand;
import com.adambots.commands.driveCommands.AngleRotateCommand;
import com.adambots.commands.driveCommands.SpinFastCommand;
import com.adambots.commands.driveCommands.StopCommand;
import com.adambots.commands.hangCommands.HangLevelCommand;
import com.adambots.commands.hangCommands.RunHangCommand;
import com.adambots.commands.hangCommands.RunLeftHangCommand;
import com.adambots.commands.hangCommands.RunRightHangCommand;
import com.adambots.commands.intakeCommands.FeedShooterCommand;
import com.adambots.commands.intakeCommands.IntakeWithAdjustCommand;
import com.adambots.commands.visionCommands.AlignRotateDriveCommand;
import com.adambots.commands.visionCommands.AprilAlignRotateCommand;
import com.adambots.commands.visionCommands.NoteAlignRotateCommand;
import com.adambots.commands.visionCommands.PathPlannerAlign;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.HangSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
  private final CANdleSubsystem ledSubsystem = new CANdleSubsystem(RobotMap.candleLEDs);
  private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.leftHangMotor, RobotMap.rightHangMotor, RobotMap.leftHangRelay, RobotMap.rightHangRelay);

  //Creates a SmartDashboard element to allow drivers to select differnt autons
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure commands to run periodically during robot operation
    setupDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Register commands for use in PathPlanner
    registerNamedCommands();

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
    //JOYSTICK BINDINGS SHOULD BE IN NUMERICAL ORDER TO PREVENT DOUBLE BINDINGS
    Buttons.JoystickButton1.whileTrue(new AdaptiveScoreCommand(armSubsystem, shooterSubsystem, intakeSubsystem));

    Buttons.JoystickButton13.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw()));
    

    //Debugging and Testing
    Buttons.JoystickButton2.onTrue(new PathPlannerAlign(drivetrainSubsystem));
    // Buttons.JoystickButton4.onTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d())));

    Buttons.JoystickButton7.whileTrue(new AlignRotateDriveCommand(drivetrainSubsystem, ledSubsystem, true, VisionConstants.aprilLimelite));
    Buttons.JoystickButton6.whileTrue(new AlignRotateDriveCommand(drivetrainSubsystem, ledSubsystem, false, VisionConstants.noteLimelite));
    Buttons.JoystickButton5.whileTrue(new AngleRotateCommand(drivetrainSubsystem, 90, RobotMap.gyro));
    Buttons.JoystickButton8.whileTrue(new SpinFastCommand(drivetrainSubsystem)); //TODO: align and drive towards it

    // Buttons.JoystickButton6.onTrue(new AprilAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5));
    // Buttons.JoystickButton8.onTrue(new NoteAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5));

    // Buttons.JoystickButton2.onTrue(new SequentialCommandGroup(new PathPlannerAlign(drivetrai\nSubsystem), new PathPlannerAlign(drivetrainSubsystem)));

    // Buttons.XboxDPadE.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.defaultState));
    // Buttons.XboxXButton.onTruef(new ChangeArmStateCommand(armSubsystem, ArmConstants.trapState));


    //Xbox Button Bindings 
    Buttons.XboxAButton.whileTrue(new IntakeWithAdjustCommand(armSubsystem, intakeSubsystem));
    Buttons.XboxAButton.onFalse(new InstantCommand(() -> VisionHelpers.offLight(VisionConstants.noteLimelite)));

    Buttons.XboxBButton.whileTrue(new HumanStationCommand(armSubsystem, intakeSubsystem));

    Buttons.XboxXButton.whileTrue(new AmpCommand(armSubsystem)); //TODO: Maybe add a slowintake

    Buttons.XboxYButton.onTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem));
    Buttons.XboxYButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    //Temporary Hang Commands
    Buttons.JoystickButton4.whileTrue(new HangLevelCommand(hangSubsystem, armSubsystem, RobotMap.gyro)); //TODO: Test carefully
    // Buttons.JoystickButton5.onTrue(new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.speakerState))); //Encoders should be reset where rods are within frame perim
    Buttons.JoystickButton9.onTrue(new InstantCommand(() -> hangSubsystem.resetEncoders())); //Encoders should be reset where rods are within frame perim
    Buttons.XboxLeftStickButton.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(1)));
    Buttons.XboxRightStickButton.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(0)));
    // Buttons.XboxLeftStickButton.onTrue(new InstantCommand(() -> hangSubsystem.setSolenoids(true)));
    // Buttons.XboxRightStickButton.onTrue(new InstantCommand(() -> hangSubsystem.setSolenoids(false)));



    //THESE COMMANDS DO NOT AUTO ENGAGE SOLENOIDS - which is why they are negative, where the solenoid should be left unpowered
    Buttons.XboxLeftBumper.whileTrue(new RunLeftHangCommand(hangSubsystem, -0.25)); //Run left winch in 
    Buttons.XboxRightBumper.whileTrue(new RunRightHangCommand(hangSubsystem, -0.25)); //Run right winch in

    //These commands do automatically engage solenoids if you are running the winches out (and leaves time for solenoids to engage)
    Buttons.XboxBackButton.whileTrue(new RunHangCommand(hangSubsystem, 1)); //Drops robot down (auto engages solenoids)
    Buttons.XboxStartButton.whileTrue(new RunHangCommand(hangSubsystem, -0.25)); //Lifts robot up


    //Xbox DPad Bindings
    Buttons.XboxDPadN.whileTrue(new RotateShoulderCommand(armSubsystem,1, true));
    Buttons.XboxDPadS.whileTrue(new RotateShoulderCommand(armSubsystem, -1, true));
    
    Buttons.XboxDPadE.whileTrue(new RotateWristCommand(armSubsystem, -0.5, true));
    Buttons.XboxDPadW.whileTrue(new RotateWristCommand(armSubsystem, 0.5, true));
  }


  private void registerNamedCommands() {
    NamedCommands.registerCommand("PrimeShooterCommand", new PrimeShooterCommand(armSubsystem, shooterSubsystem));

    NamedCommands.registerCommand("FeedShooterCloseCommand", new FeedShooterCommand(intakeSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("FeedShooterFarCommand", new FeedShooterCommand(intakeSubsystem, shooterSubsystem));
    
    NamedCommands.registerCommand("AprilAlignCommand", new ParallelRaceGroup(new WaitCommand(1), new AprilAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5)));
    NamedCommands.registerCommand("NoteAlignCommand", new ParallelRaceGroup(new WaitCommand(1), new NoteAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5)));

    NamedCommands.registerCommand("CenterFloorIntakeCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, ArmConstants.centerFloorShootState));
    NamedCommands.registerCommand("TopFloorIntakeCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, ArmConstants.topFloorShootState));

    NamedCommands.registerCommand("PrintCommand",new PrintCommand("Path Following Finished"));
    NamedCommands.registerCommand("StopCommand",new StopCommand(drivetrainSubsystem));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("AprilTagField", Constants.aprilTagfield);   
    SmartDashboard.putData("Field", Constants.field);

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());
    Dash.add("pitch", () -> RobotMap.gyro.getPitch());
    Dash.add("roll", () -> RobotMap.gyro.getRoll());

    // Dash.add("IntakeSpeed", () -> intakeSubsystem.getIntakeSpeed());

    Dash.add("ClassName", VisionHelpers.getClassName(VisionConstants.noteLimelite));
    // Dash.add("XLocation", () -> VisionHelpers.getXLocation());
    // Dash.add("YLocation", () -> VisionHelpers.getYLocation());
    Dash.add("Detected", () -> VisionHelpers.isDetected(VisionConstants.noteLimelite));
    Dash.add("VertAngle", () -> VisionHelpers.getVertAngle(VisionConstants.noteLimelite));
    Dash.add("HorizAngle", () -> VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    Dash.add("Aligned", () -> VisionHelpers.isAligned(VisionConstants.noteLimelite));
    Dash.add("DistanceAligned", () -> VisionHelpers.isDistanceAligned(VisionConstants.noteLimelite));
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
        () -> intakeSubsystem.setGroundIntakeMotorSpeed(
          Buttons.deaden(Buttons.XboxController.getRightY(),GamepadConstants.kDeadZone) * 0.12), 
        intakeSubsystem));
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
