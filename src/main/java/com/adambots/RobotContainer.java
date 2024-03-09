package com.adambots;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.ShooterConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.armCommands.AmpCommand;
import com.adambots.commands.armCommands.PrimeShooterCommand;
import com.adambots.commands.armCommands.PrimeShooterCommandFeed;
import com.adambots.commands.armCommands.PrimeShooterCommandFloor;
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
import com.adambots.commands.intakeCommands.AdaptiveScoreCommand;
import com.adambots.commands.intakeCommands.FancyAdjustCommand;
import com.adambots.commands.intakeCommands.FeedShooterCommand;
import com.adambots.commands.intakeCommands.IntakeCommand;
import com.adambots.commands.intakeCommands.IntakeWithAdjustCommand;
import com.adambots.commands.intakeCommands.SlowOuttakeCommand;
import com.adambots.commands.visionCommands.AlignRotateDriveCommand;
import com.adambots.commands.visionCommands.AprilAlignRotateCommand;
import com.adambots.commands.visionCommands.DriveToNoteCommand;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs, shooterSubsystem);
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

  public void TeleopInit() {
    VisionHelpers.setPipeline(VisionConstants.noteLimelite, 0);

    shooterSubsystem.setTargetWheelSpeed(0);

    if (DriverStation.isFMSAttached()) {
      armSubsystem.setCurrentState(ArmConstants.defaultState);
      intakeSubsystem.setLockOut(false);
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && DriverStation.isFMSAttached()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        RobotMap.gyro.setYawOffset(180);
      };
    }
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

    /* Primary bindings:
     * Adaptive Score
     * Reset Yaw (Should be a side button, shouldnt be needed for comp)
     * AlignRotateDriveCommand (to apriltag and note) - Should we make adaptive depending on arm state?
     * AngleRotateCommand (To human player station)
     * SpinFastCommand (for defense)
     * 
     * Secondary Bindings:
     * Intake
     * Human station
     * Amp
     * Prime Shooter
     * 
     * Hang level
     * Drop from hang
     * 
     * Manual Adjusts
     */

    Buttons.JoystickButton1.whileTrue(new AdaptiveScoreCommand(armSubsystem, shooterSubsystem, intakeSubsystem)); //Score in amp and speaker

    Buttons.JoystickButton2.onTrue(new PathPlannerAlign(drivetrainSubsystem)); //Drive to apriltag

    Buttons.JoystickButton13.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw())); //Reset Gyro
    
    //Both lock rotation to apriltag with driver control
    Buttons.JoystickButton7.whileTrue(new AlignRotateDriveCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, true, VisionConstants.aprilLimelite));
    Buttons.JoystickButton6.whileTrue(new AlignRotateDriveCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, true, VisionConstants.aprilLimelite));

    Buttons.JoystickButton5.whileTrue(new AngleRotateCommand(drivetrainSubsystem, -90, RobotMap.gyro)); //Rotate to amp
    Buttons.JoystickButton8.whileTrue(new SpinFastCommand(drivetrainSubsystem)); //Spin while drive driving (defense)

    Buttons.JoystickButton10.whileTrue(new AngleRotateCommand(drivetrainSubsystem, 60, RobotMap.gyro)); //Rotate to huaman station

    Buttons.JoystickButton3.whileTrue(new AngleRotateCommand(drivetrainSubsystem, -150, RobotMap.gyro)); //Rotate to shoot across field

    Buttons.JoystickButton4.whileTrue(new HangLevelCommand(hangSubsystem, armSubsystem, RobotMap.gyro)); //Hang on the chain


    //Xbox Button Bindings 
    Buttons.XboxAButton.whileTrue(new IntakeWithAdjustCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem)); //Intake off floor
    Buttons.XboxAButton.onFalse(new FancyAdjustCommand(intakeSubsystem)); //Adjust fully intaked note

    Buttons.XboxStartButton.whileTrue(new PrimeShooterCommandFeed(armSubsystem, shooterSubsystem, intakeSubsystem, ShooterConstants.mediumSpeed)); //Raise arm to human station
    Buttons.XboxStartButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    Buttons.XboxBButton.whileTrue(new PrimeShooterCommandFloor(armSubsystem, shooterSubsystem, intakeSubsystem, ShooterConstants.lowSpeed)); //Floor state and spin shooter
    Buttons.XboxBButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    Buttons.XboxXButton.whileTrue(new AmpCommand(armSubsystem)); //Move arm to amp pos

    Buttons.XboxYButton.onTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, ShooterConstants.lowSpeed)); //Speaker state and prime shooter
    Buttons.XboxYButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    Buttons.XboxLeftBumper.onTrue(new SlowOuttakeCommand(intakeSubsystem)); 
    Buttons.XboxRightBumper.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(0))); //Stop FLywheels

    //THESE COMMANDS DO NOT AUTO ENGAGE SOLENOIDS - which is why they are negative, where the solenoid should be left unpowered
    Buttons.XboxLeftTriggerButton.whileTrue(new RunLeftHangCommand(hangSubsystem, -0.25)); //Run left winch in 
    Buttons.XboxRightTriggerButton.whileTrue(new RunRightHangCommand(hangSubsystem, -0.25)); //Run right winch in

    //These commands do automatically engage solenoids if you are running the winches out (and leaves time for solenoids to engage)
    Buttons.XboxBackButton.whileTrue(new RunHangCommand(hangSubsystem, 1)); //Raises bendy rods up

    //Xbox DPad Bindings
    Buttons.XboxDPadN.whileTrue(new RotateShoulderCommand(armSubsystem,1, true));
    Buttons.XboxDPadS.whileTrue(new RotateShoulderCommand(armSubsystem, -1, true));
    
    Buttons.XboxDPadE.whileTrue(new RotateWristCommand(armSubsystem, -0.5, true));
    Buttons.XboxDPadW.whileTrue(new RotateWristCommand(armSubsystem, 0.5, true));


    //NOT NEEDED FOR COMP
    // Buttons.XboxRightStickButton.onTrue(new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.topFloorShootState));
    // Buttons.JoystickButton9.onTrue(new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.speakerState))); //Encoders should be reset where rods are within frame perim
    // Buttons.JoystickButton9.onTrue(new InstantCommand(() -> hangSubsystem.resetEncoders())); //Encoders should be reset where rods are within frame perim

    // Buttons.JoystickButton2.onTrue(new PathPlannerAlign(drivetrainSubsystem));

    // Buttons.JoystickButton6.onTrue(new AprilAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5));
    // Buttons.JoystickButton8.onTrue(new NoteAlignRotateCommand(drivetrainSubsystem, ledSubsystem, false, 5, 5));

    // Buttons.XboxDPadE.onTrue(new ChangeArmStateCommand(armSubsystem, ArmConstants.defaultState));
    // Buttons.XboxXButton.onTruef(new ChangeArmStateCommand(armSubsystem, ArmConstants.trapState));
  }


  private void registerNamedCommands() {
    NamedCommands.registerCommand("PrimeShooterCloseCommand", new PrimeShooterCommand(armSubsystem, shooterSubsystem, ShooterConstants.lowSpeed));
    NamedCommands.registerCommand("PrimeShooterFarCommand", new PrimeShooterCommand(armSubsystem, shooterSubsystem, ShooterConstants.highSpeed));

    NamedCommands.registerCommand("FeedShooterCommand", new FeedShooterCommand(intakeSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("DumbFeedShooterCommand", new ParallelDeadlineGroup(new WaitCommand(1), new IntakeCommand(intakeSubsystem, 0.7)).andThen(new IntakeCommand(intakeSubsystem, 0)));
    
    NamedCommands.registerCommand("AprilAlignCommand", new ParallelDeadlineGroup(new WaitCommand(1), new AprilAlignRotateCommand(drivetrainSubsystem, candleSubsytem, 0)));
    NamedCommands.registerCommand("AprilAlignTopCommand", new ParallelDeadlineGroup(new WaitCommand(1), new AprilAlignRotateCommand(drivetrainSubsystem, candleSubsytem, 0.5)));
    NamedCommands.registerCommand("NoteAlignCommand", new ParallelDeadlineGroup(new WaitCommand(1), new NoteAlignRotateCommand(drivetrainSubsystem, candleSubsytem, false, 3, 5)));
    NamedCommands.registerCommand("DriveToNoteCommand", new ParallelDeadlineGroup(new WaitCommand(3), new DriveToNoteCommand(drivetrainSubsystem, candleSubsytem)));

    NamedCommands.registerCommand("CenterFloorIntakeAndShootCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.closeFloorShootState));
    NamedCommands.registerCommand("TopFloorIntakeAndShootCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.topFloorShootState));
    NamedCommands.registerCommand("BottomFloorIntakeAndShootCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.closeFloorShootState));
    NamedCommands.registerCommand("TopFloorIntakeCloseCommand", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.defaultState));
    NamedCommands.registerCommand("LowFloorCloseCommand2", new FloorIntakeCommand(armSubsystem, intakeSubsystem, shooterSubsystem, candleSubsytem, ArmConstants.closeFloorShootState));
    NamedCommands.registerCommand("TopShootSpeakerCommand", new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.speakerState)));

    NamedCommands.registerCommand("PrintCommand",new PrintCommand("Path Following Finished"));
    NamedCommands.registerCommand("StopCommand",new StopCommand(drivetrainSubsystem));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    Dash.add("getClassName", VisionHelpers.getClassName(VisionConstants.noteLimelite));
        Dash.add("getHeartbeat", () -> VisionHelpers.getHeatbeat());


    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("AprilTagField", Constants.aprilTagfield);   
    SmartDashboard.putData("Field", Constants.field);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);

    Dash.add("getX", () -> VisionHelpers.getCameraPose2eTargetSpace().getX());
    Dash.add("getY", () -> VisionHelpers.getCameraPose2eTargetSpace().getY());
    Dash.add("getZ", () -> VisionHelpers.getCameraPose2eTargetSpace().getX());
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
    Dash.add("Detected", () -> VisionHelpers.isDetected(VisionConstants.aprilLimelite));
    Dash.add("VertAngle", () -> VisionHelpers.getVertAngle(VisionConstants.noteLimelite));
    Dash.add("HorizAngle", () -> VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    Dash.add("Aligned", () -> VisionHelpers.isAligned(VisionConstants.noteLimelite));
    Dash.add("DistanceAligned", () -> VisionHelpers.isDistanceAligned(VisionConstants.noteLimelite));
  }

  private void setupDefaultCommands() {
    drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                Buttons.rotateSupplier.getAsDouble()*DriveConstants.kTeleopRotationalSpeed,
                true),
            drivetrainSubsystem));
    intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> intakeSubsystem.setMotorSpeed(Buttons.applyCurve(Buttons.XboxController.getLeftY(), Buttons.forwardCurve) * 0.15), 
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
