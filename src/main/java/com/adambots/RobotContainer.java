package com.adambots;

import com.adambots.Constants.ArmConstants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.ShooterConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.commands.armCommands.AmpCommand;
import com.adambots.commands.armCommands.PrimeShooterCommand;
import com.adambots.commands.armCommands.RetractShooterCommand;
import com.adambots.commands.armCommands.RotateShoulderCommand;
import com.adambots.commands.armCommands.RotateWristCommand;
import com.adambots.commands.armCommands.SyncShoulderCommand;
import com.adambots.commands.driveCommands.DriveToWaypointCommand;
import com.adambots.commands.driveCommands.RotateToAngleCommand;
import com.adambots.commands.driveCommands.SpinCommand;
import com.adambots.commands.driveCommands.StopCommand;
import com.adambots.commands.hangCommands.HangLevelCommand;
import com.adambots.commands.hangCommands.RunHangCommand;
import com.adambots.commands.hangCommands.RunLeftHangCommand;
import com.adambots.commands.hangCommands.RunRightHangCommand;
import com.adambots.commands.intakeCommands.AdaptiveScoreCommand;
import com.adambots.commands.intakeCommands.AdjustNoteCommand;
import com.adambots.commands.intakeCommands.AutonIntakeCommand;
import com.adambots.commands.intakeCommands.ForceFeedShooterCommand;
import com.adambots.commands.intakeCommands.IntakeToFlywheelCommand;
import com.adambots.commands.intakeCommands.SpinFlywheelsCommand;
import com.adambots.commands.visionCommands.DriveToNoteCommand;
import com.adambots.commands.visionCommands.OdomSpeakerAlignCommand;
import com.adambots.subsystems.ArmSubsystem;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.HangSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.utils.Buttons;
import com.adambots.utils.Dash;
import com.adambots.vision.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  private final ArmSubsystem armSubsystem = new ArmSubsystem(RobotMap.shoulderMotor, RobotMap.wristMotor, RobotMap.shoulderEncoder, RobotMap.wristEncoder);
  private final CANdleSubsystem candleSubsytem = new CANdleSubsystem(RobotMap.candleLEDs);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(RobotMap.shooterWheel, RobotMap.shooterWheel2);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.groundIntakeMotor, RobotMap.firstPieceInRobotEye, RobotMap.secondPieceInRobotEye);
  private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.leftHangMotor, RobotMap.rightHangMotor, RobotMap.leftHangSolenoid, RobotMap.rightHangSolenoid);

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
    if (DriverStation.isFMSAttached()) {
      armSubsystem.setCurrentState(ArmConstants.defaultState);
      shooterSubsystem.setTargetWheelSpeed(0);
      intakeSubsystem.setLockOut(false);
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

    // Buttons.JoystickButton1.whileTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed)),
    //   new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.topFloorShootState)),
    //   new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S1_POSE2D),
    //   new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    // ));
    // Buttons.JoystickButton2.whileTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed)),
    //   new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S2_POSE2D),
    //   new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    // ));
    // Buttons.JoystickButton2.whileTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed)),
    //   new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S3_POSE2D),
    //   new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    // ));

    Buttons.JoystickButton1.whileTrue(new AdaptiveScoreCommand(armSubsystem, shooterSubsystem, intakeSubsystem)); //Score in amp and speaker
    
    Buttons.JoystickButton2.whileTrue(new DriveToNoteCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, 1.5)); //Score in amp and speaker

    Buttons.JoystickButton3.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 90, RobotMap.gyro)); //Rotate to amp
    Buttons.JoystickButton4.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, -60, RobotMap.gyro)); //Rotate to huaman station

    Buttons.JoystickButton5.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 180, RobotMap.gyro)); //Rotate to huaman station

    Buttons.JoystickButton6.whileTrue(new RotateToAngleCommand(drivetrainSubsystem, 155, RobotMap.gyro)); //Rotate to feed
    Buttons.JoystickButton6.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.feedState)); //Rotate to huaman station
    Buttons.JoystickButton6.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    // Buttons.JoystickButton7.whileTrue(new AlignWhileDrivingCommand(drivetrainSubsystem, candleSubsytem, VisionConstants.defaultAprilLimelite));
    Buttons.JoystickButton7.whileTrue(new OdomSpeakerAlignCommand(drivetrainSubsystem, armSubsystem, shooterSubsystem, candleSubsytem, VisionConstants.defaultAprilLimelite));
    // Buttons.JoystickButton7.whileTrue(new InterpolateDistanceCommand(armSubsystem, shooterSubsystem, drivetrainSubsystem, intakeSubsystem, VisionLookUpTable.defaultShooterConfig));
    // Buttons.JoystickButton7.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.defaultSpeakerState)); //Default state and prime shooter
    // Buttons.JoystickButton7.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    Buttons.JoystickButton8.whileTrue(new HangLevelCommand(hangSubsystem, armSubsystem, RobotMap.gyro, candleSubsytem)); //Hang on the chain

    Buttons.JoystickButton10.whileTrue(new SpinCommand(drivetrainSubsystem)); //Spin while drive driving (defense)

    Buttons.JoystickButton11.whileTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d(1.38, 5.53, new Rotation2d(Math.PI)))));

    Buttons.JoystickButton13.onTrue(new InstantCommand(() -> RobotMap.gyro.resetYaw())); //Reset Gyro
    
    Buttons.JoystickButton16.onTrue(new SyncShoulderCommand(armSubsystem));


    //Xbox Button Bindings 
    Buttons.XboxAButton.whileTrue(new IntakeToFlywheelCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem)); //Intake off floor
    Buttons.XboxAButton.onFalse(new AdjustNoteCommand(intakeSubsystem, shooterSubsystem)); //Adjust fully intaked note

    Buttons.XboxBButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.defaultSpeakerState)); //Speaker state and prime shooter
    Buttons.XboxBButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));

    Buttons.XboxXButton.whileTrue(new AmpCommand(armSubsystem, shooterSubsystem)); //Move arm to amp pos

    Buttons.XboxYButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.mediumSpeed, ArmConstants.speakerState)); //Speaker state and prime shooter
    Buttons.XboxYButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem)); //Default state and stop shooter


    Buttons.XboxLeftBumper.whileTrue(new SpinFlywheelsCommand(shooterSubsystem, intakeSubsystem)); //Spin up flywheels
    Buttons.XboxRightBumper.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(0))); //Stop FLywheels

    //THESE COMMANDS DO NOT AUTO ENGAGE SOLENOIDS - which is why they are negative, where the solenoid should be left unpowered
    Buttons.XboxLeftTriggerButton.whileTrue(new RunLeftHangCommand(hangSubsystem, -0.25)); //Run left winch in 
    Buttons.XboxRightTriggerButton.whileTrue(new RunRightHangCommand(hangSubsystem, -0.25)); //Run right winch in


    Buttons.XboxRightStickButton.onTrue(new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(50)));

    //These commands do automatically engage solenoids if you are running the winches out (and leaves time for solenoids to engage)
    Buttons.XboxBackButton.whileTrue(new RunHangCommand(hangSubsystem, candleSubsytem, 1)); //Raises bendy rods up

    Buttons.XboxStartButton.whileTrue(new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.highSpeed, ArmConstants.closeFloorShootState));
    Buttons.XboxStartButton.onFalse(new RetractShooterCommand(armSubsystem, shooterSubsystem));


    //Xbox DPad Bindings
    Buttons.XboxDPadN.whileTrue(new RotateShoulderCommand(armSubsystem,1, true));
    Buttons.XboxDPadS.whileTrue(new RotateShoulderCommand(armSubsystem, -1, true));
    
    Buttons.XboxDPadE.whileTrue(new RotateWristCommand(armSubsystem, -0.5, true));
    Buttons.XboxDPadW.whileTrue(new RotateWristCommand(armSubsystem, 0.5, true));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("ShootPreload", new SequentialCommandGroup(
      new PrimeShooterCommand(armSubsystem, shooterSubsystem, intakeSubsystem, candleSubsytem, ShooterConstants.mediumSpeed, ArmConstants.speakerState),
      new WaitCommand(1),
      new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem),
      new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.closeFloorShootState))
    ));
    NamedCommands.registerCommand("IntakeNote->ShootState", new SequentialCommandGroup(
      new AutonIntakeCommand(armSubsystem, intakeSubsystem, candleSubsytem),
      new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.closeFloorShootState))
    ));
    NamedCommands.registerCommand("SpinUpShooter",
      new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed))
    );
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
      new InstantCommand(() -> drivetrainSubsystem.stop()),
      new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));

    NamedCommands.registerCommand("DriveToNote", new DriveToNoteCommand(drivetrainSubsystem, intakeSubsystem, candleSubsytem, 2));

    NamedCommands.registerCommand("StopCommand", new StopCommand(drivetrainSubsystem));

    NamedCommands.registerCommand("S1Approach->Score", new SequentialCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed)),
      new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.topFloorShootState)),
      new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S1_POSE2D),
      new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("S2Approach->Score", new SequentialCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.mediumSpeed)),
      new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.closeFloorShootState)),
      new ParallelDeadlineGroup(
        new WaitCommand(3.5), 
        new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S2_POSE2D)
      ),
      new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
    NamedCommands.registerCommand("S3Approach->Score", new SequentialCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTargetWheelSpeed(ShooterConstants.highSpeed)),
      new InstantCommand(() -> armSubsystem.setCurrentState(ArmConstants.topFloorShootState)),
      new DriveToWaypointCommand(drivetrainSubsystem, RobotMap.gyro, AutoConstants.S3_POSE2D),
      new ForceFeedShooterCommand(intakeSubsystem, shooterSubsystem)
    ));
  }

  private void setupDashboard() {    
    autoChooser = AutoBuilder.buildAutoChooser();

    //Adds various data to the dashboard that is useful for driving and debugging
    SmartDashboard.putData("Auton Mode", autoChooser);

    // SmartDashboard.putData("FrontLL Field", Constants.frontLLField);   
    // SmartDashboard.putData("RearLL Field", Constants.rearLLField);   
    SmartDashboard.putData("Odom Field", Constants.odomField);

    // Dash.add("getY", Buttons.forwardSupplier);
    // Dash.add("getX", Buttons.sidewaysSupplier);
    // Dash.add("getZ", Buttons.rotateSupplier);

    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("Trigger", Buttons.JoystickButton1);

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());

    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());
    Dash.add("pitch", () -> RobotMap.gyro.getPitch());
    Dash.add("roll", () -> RobotMap.gyro.getRoll());

    Dash.add("isAtTarSpeed", () -> shooterSubsystem.isAtTargetSpeed());
    Dash.add("isAtTarState", () -> armSubsystem.isAtTargetState());
    Dash.add("distToSpeaker", () -> VisionHelpers.getAprilHorizDist(VisionConstants.defaultAprilLimelite));
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
        () -> intakeSubsystem.setMotorSpeed(Buttons.applyCurve(Buttons.XboxController.getLeftY(), Buttons.forwardCurve) * 0.25), 
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
//Blahaj_Counter: 4
