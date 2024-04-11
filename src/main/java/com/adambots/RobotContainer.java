package com.adambots;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.commands.driveCommands.RotateToAngleCommand;
import com.adambots.commands.driveCommands.SpinCommand;
import com.adambots.commands.driveCommands.StopCommand;
import com.adambots.commands.driveCommands.VisionDriveToWaypointCommand;
// import com.adambots.subsystems.ArmSubsystem;
// import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
// import com.adambots.subsystems.HangSubsystem;
// import com.adambots.subsystems.IntakeSubsystem;
// import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.utils.Dash;
import com.adambots.vision.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  // private final ArmSubsystem armSubsystem = new ArmSubsystem(RobotMap.shoulderMotor, RobotMap.wristMotor, RobotMap.shoulderEncoder, RobotMap.wristEncoder);
  // private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs);
  // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(RobotMap.shooterWheel, RobotMap.shooterWheel2);
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.groundIntakeMotor, RobotMap.firstPieceInRobotEye, RobotMap.secondPieceInRobotEye);
  // private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.leftHangMotor, RobotMap.rightHangMotor, RobotMap.leftHangSolenoid, RobotMap.rightHangSolenoid);

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

  public void teleopInit() {
    // new VisionOdomResetCommand(drivetrainSubsystem, VisionConstants.defaultAprilLimelite).schedule();
    // new VisionOdomResetCommand(drivetrainSubsystem, VisionConstants.aprilLimelite).schedule();


    // shooterSubsystem.setTargetWheelSpeed(0);

    // if (DriverStation.isFMSAttached()) {
    //   armSubsystem.setCurrentState(ArmConstants.defaultState);
    //   intakeSubsystem.setLockOut(false);
    // }

    if (Robot.isOnRedAlliance() && DriverStation.isFMSAttached()) {
      RobotMap.gyro.offsetYawByAngle(180);
    }

    // new VisionOdomResetCommand(drivetrainSubsystem).schedule();
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

    // Buttons.JoystickButton1.whileTrue(new AdaptiveScoreCommand(armSubsystem, shooterSubsystem, intakeSubsystem)); //Score in amp and speaker
    Buttons.JoystickButton1.whileTrue(new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(0.78, 6.64), new Rotation2d(Math.toRadians(-120))))); //Score in amp and speaker
    Buttons.JoystickButton2.whileTrue(new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(1.38, 5.43), new Rotation2d(Math.toRadians(180))))); //Score in amp and speaker
    Buttons.JoystickButton3.whileTrue(new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(0.78, 4.25), new Rotation2d(Math.toRadians(120))))); //Score in amp and speaker
    
    // Buttons.JoystickButton2.whileTrue(new DriveToNoteCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, 3)); //Score in amp and speaker
    
    // // Buttons.JoystickButton6.whileTrue(new DriveToNoteCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, 1)); //Score in amp and speaker

    // Buttons.JoystickButton5.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 153, RobotMap.gyro)); //Rotate to huaman station
    // Buttons.JoystickButton5.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.defaultPodiumState)); //Speaker state and prime shooter
    // Buttons.JoystickButton5.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));
    // Buttons.JoystickButton5.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(90))); //Stop FLywheels


    // Buttons.JoystickButton6.whileTrue(new OdomSpeakerAlignCommand(drivetrainSubsystem, armSubsystem, shooterSubsystem, candleSubsytem, VisionConstants.defaultAprilLimelite));
    Buttons.JoystickButton6.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 180, RobotMap.gyro)); //Rotate to huaman station

    //Both lock rotation to apriltag with driver control

    // Buttons.JoystickButton3.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 90, RobotMap.gyro)); //Rotate to amp
    Buttons.JoystickButton4.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, -60, RobotMap.gyro)); //Rotate to huaman station

    Buttons.JoystickButton10.whileTrue(new SpinCommand(drivetrainSubsystem)); //Spin while drive driving (defense)

    // Buttons.JoystickButton9.whileTrue(new OdomSpeakerAlignCommand(drivetrainSubsystem, armSubsystem, shooterSubsystem, candleSubsytem, VisionConstants.aprilLimelite));
    // Buttons.JoystickButton9.whileTrue(new VisionOdomResetCommand(drivetrainSubsystem, VisionConstants.aprilLimelite));

    // Buttons.JoystickButton8.whileTrue(new HangLevelCommand(hangSubsystem, armSubsystem, RobotMap.gyro, candleSubsytem)); //Hang on the chain

    // Buttons.JoystickButton7.whileTrue(new AlignWhileDrivingCommand(drivetrainSubsystem, candleSubsytem, VisionConstants.defaultAprilLimelite));
    // Buttons.JoystickButton7.whileTrue(new InterpolateDistanceCommand(armSubsystem, shooterSubsystem, drivetrainSubsystem, intakeSubsystem, VisionLookUpTable.defaultShooterConfig));
    // Buttons.JoystickButton7.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    //TODO: Uncomment
    // Buttons.JoystickButton7.whileTrue(new OdomSpeakerAlignCommand(drivetrainSubsystem, armSubsystem, shooterSubsystem, candleSubsytem, VisionConstants.defaultAprilLimelite));


    Buttons.JoystickButton13.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw())); //Reset Gyro

    // Buttons.JoystickButton10.whileTrue(new ShootWhenAligned(drivetrainSubsystem, candleSubsytem, intakeSubsystem, armSubsystem, shooterSubsystem));


    Buttons.JoystickButton11.whileTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d(1.38, 5.53, new Rotation2d(0)))));
    
    // Buttons.JoystickButton16.onTrue(new SyncShoulderCommand(armSubsystem));

    // Buttons.JoystickButton15.onTrue(new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem));


    // Buttons.JoystickButton3.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, -150, RobotMap.gyro)); //Rotate to shoot across field


    //Xbox Button Bindings 
    // Buttons.XboxAButton.whileTrue(new IntakeToFlywheelCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem)); //Intake off floor
    // Buttons.XboxAButton.onFalse(new AdjustNoteCommand(intakeSubsystem)); //Adjust fully intaked note

    // // Buttons.XboxStartButton.whileTrue(new PrimeShooterCommandFeed(armSubsystem, shooterSubsystem, intakeSubsystem, ShooterConstants.mediumSpeed)); //Raise arm to human station
    // // Buttons.XboxStartButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    // // Buttons.XboxBButton.whileTrue(new InterpolateDistanceCommand(armSubsystem, shooterSubsystem, drivetrainSubsystem, intakeSubsystem, VisionLookUpTable.lowShooterConfig));
    // Buttons.XboxBButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.defaultSpeakerState)); //Speaker state and prime shooter
    // Buttons.XboxBButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));
    
    // Buttons.XboxStartButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.closeFloorShootState));
    // Buttons.XboxStartButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));
    // Buttons.XboxStartButton.whileTrue(new VisionOdomResetCommand(drivetrainSubsystem));

    // Buttons.XboxBButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, ShooterConstants.mediumSpeed, ArmConstants.closeFloorShootState)); //Floor state and spin shooter
    // Buttons.XboxBButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    // Buttons.XboxXButton.whileTrue(new AmpCommand(armSubsystem)); //Move arm to amp pos

    // Buttons.XboxYButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.mediumSpeed, ArmConstants.speakerState)); //Speaker state and prime shooter
    // Buttons.XboxYButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter

    // // Buttons.XboxLeftBumper.onTrue(new SlowOuttakeCommand(intakeSubsystem)); 
    // Buttons.XboxRightBumper.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(0))); //Stop FLywheels

    // Buttons.XboxLeftBumper.whileTrue(new SpinFlywheelsCommand(shooterSubsystem, intakeSubsystem)); //Spin up flywheels

    // //THESE COMMANDS DO NOT AUTO ENGAGE SOLENOIDS - which is why they are negative, where the solenoid should be left unpowered
    // Buttons.XboxLeftTriggerButton.whileTrue(new RunLeftHangCommand(hangSubsystem, -0.25)); //Run left winch in 
    // Buttons.XboxRightTriggerButton.whileTrue(new RunRightHangCommand(hangSubsystem, -0.25)); //Run right winch in

    // Buttons.XboxRightStickButton.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(50)));

    // //These commands do automatically engage solenoids if you are running the winches out (and leaves time for solenoids to engage)
    // Buttons.XboxBackButton.whileTrue(new RunHangCommand(hangSubsystem, candleSubsytem, 1)); //Raises bendy rods up

    // //Xbox DPad Bindings
    // Buttons.XboxDPadN.whileTrue(new RotateShoulderCommand(armSubsystem,1, true));
    // Buttons.XboxDPadS.whileTrue(new RotateShoulderCommand(armSubsystem, -1, true));
    
    // Buttons.XboxDPadE.whileTrue(new RotateWristCommand(armSubsystem, -0.5, true));
    // Buttons.XboxDPadW.whileTrue(new RotateWristCommand(armSubsystem, 0.5, true));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("ShootPreload", new SequentialCommandGroup(
      // new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.mediumSpeed, ArmConstants.speakerState),
      // new WaitCommand(1),
      // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("IntakeNote->ShootState", new SequentialCommandGroup(
      // new AutonIntakeCommand(armSubsystem, intakeSubsystem, candleSubsytem),
      // new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.closeFloorShootState))
    ));
    NamedCommands.registerCommand("SpinUpShooter",
    new WaitCommand(0.1)
      // new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed))
    );
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
      // new InstantCommand(() -> drivetrainSubsystem.stop()),
      // new WaitCommand(0.3),
      // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("S1Approach->Score", new SequentialCommandGroup(
      // new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed)),
      new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(0.71, 6.70), new Rotation2d(Math.toRadians(60))))
      // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("S2Approach->Score", new SequentialCommandGroup(
      // new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed)),
      new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(1.38, 5.53), new Rotation2d(Math.toRadians(0))))
      // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("S3Approach->Score", new SequentialCommandGroup(
      // new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed)),
      new VisionDriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, new Pose2d(new Translation2d(0.78, 4.25), new Rotation2d(Math.toRadians(120))))
      // new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));

    // NamedCommands.registerCommand("DriveToNote", new DriveToNoteCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, 1.5));

    NamedCommands.registerCommand("StopCommand", new StopCommand(drivetrainSubsystem));

    NamedCommands.registerCommand("NF1_OdomReset", new InstantCommand(() -> drivetrainSubsystem.resetOdometryXY(new Translation2d(8.28, 7.44))));
    NamedCommands.registerCommand("NF2_OdomReset", new InstantCommand(() -> drivetrainSubsystem.resetOdometryXY(new Translation2d(8.28, 5.77))));
    // NamedCommands.registerCommand("NF4_OdomReset", new InstantCommand(() -> drivetrainSubsystem.resetOdometryXY(new Translation2d(8.28, 2.44))));
    NamedCommands.registerCommand("NF4_OdomReset", new InstantCommand());
    NamedCommands.registerCommand("NF5_OdomReset", new InstantCommand());
    // NamedCommands.registerCommand("NF5_OdomReset", new InstantCommand(() -> drivetrainSubsystem.resetOdometryXY(new Translation2d(8.28, 0.77))));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);
    SmartDashboard.putData("AprilTagField", Constants.aprilTagfield);   
    SmartDashboard.putData("OdomField", Constants.field);
    SmartDashboard.putData("DebugField", Constants.debugField);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);

    // Dash.add("getX", () -> VisionHelpers.getCameraPoseTargetSpace().getX());
    // Dash.add("getY", () -> VisionHelpers.getCameraPoseTargetSpace().getY());
    // Dash.add("getZ", () -> VisionHelpers.getCameraPoseTargetSpace().getZ());
    // Dash.add("newAprilDegreesAngle", () -> VisionHelpers.getHorizAngle(VisionConstants.aprilLimelite));
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());
    Dash.add("pitch", () -> RobotMap.gyro.getPitch());
    Dash.add("roll", () -> RobotMap.gyro.getRoll());

    Dash.add("Vision X", () -> VisionHelpers.getHorizAngle(VisionConstants.aprilLimelite));
    // Dash.add("Vision y", () -> VisionHelpers.getAprilTagBotPose2dBlue(VisionConstants.aprilLimelite).getY());
    // Dash.add("rOATION y", () -> VisionHelpers.getAprilTagBotPose2dBlue().getRotation().getDegrees());


    // Dash.add("IntakeSpeed", () -> intakeSubsystem.getIntakeSpeed());

    // Dash.add("isAtTargetSpeed", () -> shooterSubsystem.isAtTargetSpeed());
    // Dash.add("isAtTargetState", () -> armSubsystem.isAtTargetState());
    // Dash.add("getDistancetoSpeaker", () -> VisionHelpers.getAprilHorizDist(VisionConstants.defaultAprilLimelite));

    // Dash.add("ClassName", VisionHelpers.getClassName(VisionConstants.noteLimelite));
    // Dash.add("getHeartbeat", () -> VisionHelpers.getHeartbeat(VisionConstants.noteLimelite));
    // Dash.add("XLocation", () -> VisionHelpers.getXLocation());
    // Dash.add("YLocation", () -> VisionHelpers.getYLocation());
    // Dash.add("Detected", () -> VisionHelpers.isDetected(VisionConstants.aprilLimelite));
    // Dash.add("VertAngle", () -> VisionHelpers.getVertAngle(VisionConstants.noteLimelite));
    // Dash.add("HorizAngle", () -> VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    // Dash.add("Aligned", () -> VisionHelpers.isAligned(VisionConstants.noteLimelite, 3));
  }

  private void setupDefaultCommands() {
    drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                Buttons.forwardSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                Buttons.sidewaysSupplier.getAsDouble()*DriveConstants.kMaxSpeedMetersPerSecond,
                // 0,
                Buttons.rotateSupplier.getAsDouble()*DriveConstants.kTeleopRotationalSpeed,
                // 0,
                true),
            drivetrainSubsystem));

    // intakeSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> intakeSubsystem.setMotorSpeed(Buttons.applyCurve(Buttons.XboxController.getLeftY(), Buttons.forwardCurve) * 0.2), 
    //     intakeSubsystem));
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
//Blahaj_Counter: 4
