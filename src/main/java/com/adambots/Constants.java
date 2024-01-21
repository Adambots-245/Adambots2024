/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final String kDefaultShuffleboardTab = "debug";
    public static Field2d field = new Field2d();

    public static final class DriveConstants {

        public static final boolean kFrontLeftDriveEncoderReversed = true; //All modules are reversed so that robot movement matches odometry movement
        public static final boolean kRearLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kRearRightDriveEncoderReversed = true;

        // Distance between centers of right and left wheels on robot in meters
        public static final double kTrackWidth = 0.61;
        // Distance between front and back wheels on robot in meters
        public static final double kWheelBase = 0.61;

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
      
        public static final Map<ModulePosition, Translation2d> kModuleTranslations = Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            ModulePosition.REAR_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.REAR_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
      
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxSpeedMetersPerSecond = 4.6; //Max speed of the robot in m/s, used in teleop and auton (should be set to real world value)
        public static final double kTeleopRotationalSpeed = 10; //Rotational speed factor of the robot to be used for the teleop drive command
    }

    public static final class ModuleConstants {
        public static final double kWheelRadiusMeters = 0.047625;
        public static final double kSwerveModuleFinalGearRatio = 6.75;
        public static final double kDriveEncoderDistancePerRPM =  (Math.PI/30) / kSwerveModuleFinalGearRatio * Constants.ModuleConstants.kWheelRadiusMeters;
        // Convert drive motor rpm to linear wheel speed          RPM to rad/s    Motor rad/s to Wheel rad/s            Wheel rad/s to linear m/s 
            
        public static final double kDriveEncoderScale = 0.0470915; //Tuned value that corrosponds wheel encoders to real distance

        public static final double kPModuleDriveController = 0.3; //PID Values for drive motors
        public static final double kDModuleDriveController = 0.016;

        public static final double kPModuleTurningController = 1.1; //PID Values for turning motors (not drive motors)
        public static final double kIModuleTurningController = 0;
        public static final double kDModuleTurningController = 0.01;

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 16 * Math.PI; //Limits for wheel turning profiled PID
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 16 * Math.PI;
    }

    public static final class AutoConstants {
        public static final double kPTranslationController = 8; // PD values for auton X, Y translational movement
        public static final double kDTranslationController = 0.03;

        public static final double kPThetaController = 1.1; // PD values for auton rotational movement
        public static final double kDThetaController = 0.01;

        public static final double kDrivebaseRadius = 4.5; // Drive base radius in meters. Distance from robot center to furthest module.
    }

    public static final class GamepadConstants {
        public static final double kDeadZone = 0.15; //Deadzone applied when using Xbox controller joysticks - does not apply to big Joystick
    
        public static final int kDpadNAngle = 0;
        public static final int kDpadNEAngle = 45;
        public static final int kDpadEAngle = 90;
        public static final int kDpadSEAngle = 135;
        public static final int kDpadSAngle = 180;
        public static final int kDpadSWAngle = 225;
        public static final int kDpadWAngle = 270;
        public static final int kDpadNWAngle = 315;
    }
}
