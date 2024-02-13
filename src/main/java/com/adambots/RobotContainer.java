package com.adambots;

import java.util.List;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.Gamepad.Buttons;
// import com.adambots.commands.autonCommands.AlignNoteBothCommand;
import com.adambots.commands.autonCommands.AlignRotateCommand;
import com.adambots.commands.autonCommands.PathPlannerAlign;
import com.adambots.commands.autonCommands.TestDriveToAprilTagCommand;
import com.adambots.commands.autonCommands.autonCommandGrounds.PickupGamepieceRotateCommand;
import com.adambots.commands.autonCommands.autonCommandGrounds.PickupGamepieceStrafeCommand;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.utils.Dash;
import com.adambots.utils.VisionHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    Buttons.JoystickButton2.whileTrue(Commands.runOnce(() -> {
      Pose2d currentPose = drivetrainSubsystem.getPose();
      Pose2d currentAprilPose = VisionHelpers.getAprilTagPose2d();
      // The rotation component in these poses represents the direction of travel
      // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(currentAprilPose.getTranslation().getX()-1.23, currentAprilPose.getTranslation().getY() - 2.55)), new Rotation2d());
      // Pose2d endPos = new Pose2d(new Translation2d(6.7, 1.49), new Rotation2d(0));

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          5, 5, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, new Rotation2d(Math.toRadians(270)))
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));

    //Debugging and Testing
    Buttons.JoystickButton4.onTrue(new InstantCommand(() -> drivetrainSubsystem.resetOdometry(new Pose2d())));
    // Buttons.JoystickButton7.onTrue(new AlignNoteBothCommand(drivetrainSubsystem));
    // Buttons.JoystickButton7.onTrue(Commands.parallel(new AlignNoteHorizCommand(drivetrainSubsystem),
    //     new AlignNoteVertCommand(drivetrainSubsystem)));

    Buttons.JoystickButton6.onTrue(new PickupGamepieceStrafeCommand(drivetrainSubsystem));
        Buttons.JoystickButton7.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.aprilLimelite));

    Buttons.JoystickButton5.whileTrue(new AlignRotateCommand(drivetrainSubsystem, true, true, VisionConstants.noteLimelite));
    Buttons.JoystickButton10.whileTrue(new AlignRotateCommand(drivetrainSubsystem, false, true, VisionConstants.noteLimelite));
    Buttons.JoystickButton9.whileTrue(new PickupGamepieceRotateCommand(drivetrainSubsystem));
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


  //  Dash.add("xPID", () -> AlignNoteCommand);

    Dash.add("getY", Buttons.forwardSupplier);
    Dash.add("getX", Buttons.sidewaysSupplier);
    Dash.add("getZ", Buttons.rotateSupplier);
    Dash.add("getRawZ", () -> Buttons.ex3dPro.getZ());

    Dash.add("odom x", () -> drivetrainSubsystem.getPose().getX());
    Dash.add("odom y", () -> drivetrainSubsystem.getPose().getY());
    Dash.add("yaw", () -> RobotMap.gyro.getContinuousYawDeg());

    Dash.add("ClassName", VisionHelpers.getClassName(VisionConstants.noteLimelite));
    // Dash.add("XLocation", () ->VisionHelpers.getXLocation());
    // Dash.add("YLocation", () ->VisionHelpers.getYLocation());
    Dash.add("Detected", () ->VisionHelpers.isDetected(VisionConstants.noteLimelite));
    Dash.add("VertAngle", () ->VisionHelpers.getVertAngle(VisionConstants.noteLimelite));
    Dash.add("HorizAngle", () ->VisionHelpers.getHorizAngle(VisionConstants.noteLimelite));
    Dash.add("Aligned", () ->VisionHelpers.isAligned(VisionConstants.noteLimelite));
    Dash.add("DistanceAligned", () ->VisionHelpers.isDistanceAligned(VisionConstants.noteLimelite));

    // Dash.add("getXVision", () -> VisionHelpers.getCameraPoseTargetSpace(VisionConstants.aprilLimelite).getX());
    // Dash.add("getYVision", () -> VisionHelpers.getCameraPoseTargetSpace(VisionConstants.aprilLimelite).getY());
    // Dash.add("getZVision", () -> VisionHelpers.getCameraPoseTargetSpace(VisionConstants.aprilLimelite).getZ());
    // Dash.add("robotPosVisionX", () -> VisionHelpers.getAprilTagPose2d().getX());
    // Dash.add("robotPosVisionY", () -> VisionHelpers.getAprilTagPose2d().getY());


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
