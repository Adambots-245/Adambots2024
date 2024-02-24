package com.adambots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;

/**
 * All constant values for robot operation - Any ports should be defined in {@link RobotMap} 
 */
public final class Constants {
    public static final String kDefaultShuffleboardTab = "debug";
    public static Field2d field = new Field2d();    
    public static Field2d aprilTagfield = new Field2d();

    public static final String CANivoreBus = "CANivore";

    public static final class LEDConstants {
        public static final int LEDS_IN_STRIP = 200;

        public static final Color adambotsYellow = new Color(255, 216, 0);
        public static final Color yellow = new Color(255, 150, 0);
        public static final Color blue = new Color(0, 0, 255);
        public static final Color orange = new Color(255, 90, 5);
    }

    public static final class VisionConstants {
        public static final Pose2d aprilTagRedPose2d = new Pose2d(new Translation2d(1.23, 2.55), new Rotation2d());
        public static final String noteLimelite = "limelight-notebot";
        public static final String aprilLimelite = "limelight-april";
        public static final double kpBothPID = 0.1;
        public static final double kdBothPID = 0.035;

        public static final double kpHorizPID = 0.1;
        public static final double kdHorizPID = 0.02;

        public static final double kpRotatePID = 0.08;
        public static final double kdRotatePID = 0.025;

        public static final double kpVertPID = 0.1;
        public static final double kdVertPID = 0.02;

        public static final double kPThetaController = 5; 
        public static final double kDThetaController = 0.12;
    }

    public static final class DriveConstants {
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        // Distance between centers of right and left wheels on robot in meters
        public static final double kTrackWidth = 0.61;
        // Distance between front and back wheels on robot in meters
        public static final double kWheelBase = 0.61;
        // Drive base radius in meters. Distance from robot center to furthest module, hypotenuse of kTrackWidth/2 and kWheelBase/2
        public static final double kDrivebaseRadius = Math.hypot(kTrackWidth/2, kWheelBase/2);

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
      
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        //Max speed of the robot in m/s, used in teleop and auton (should be set to real world value)
        public static final double kMaxSpeedMetersPerSecond = 4.35; 
        //Rotational speed factor in rad/s of the robot to be used for the teleop drive command
        public static final double kTeleopRotationalSpeed = 10; 
    }

    public static final class ModuleConstants {
        public static final double kWheelRadiusMeters = 0.047625; //Should be as precise as you can get it
        public static final double kSwerveModuleFinalGearRatio = 1/6.75; //Google the swerve module model to find this value

        // Convert drive motor rpm to linear wheel speed                  Motor RPM to Wheel RPM -> RPM to rad/s -> Wheel rad/s to linear m/s 
        public static final double kDriveEncoderVelocityConversionFactor = kSwerveModuleFinalGearRatio * (Math.PI/30) * kWheelRadiusMeters;

        // Convert drive motor rotations to linear distance             Motor rot to Wheel rot -> Wheel rot to linear meters (circumference)
        public static final double kDriveEncoderPositionConversionFactor = kSwerveModuleFinalGearRatio * 2*Math.PI * kWheelRadiusMeters;

        public static final double kPModuleTurningController = 0.7; //PID Values for turning motors
        public static final double kDModuleTurningController = 0.026;
    }

    public static final class AutoConstants {
        // PD values for auton X, Y translational movement
        public static final double kPTranslationController = 11; 
        public static final double kDTranslationController = 0.11;

        // PD values for auton rotational movement
        public static final double kPThetaController = 1.7; 
        public static final double kDThetaController = 0.01;
    }

    public static final class GamepadConstants {
        // Xbox controller joystick deadzone
        public static final double kDeadZone = 0.15;
    
        // DPad returns an angle in degrees 0-315 at 45 intervals
        public static final int kDpadNAngle = 0;
        public static final int kDpadNEAngle = 45;
        public static final int kDpadEAngle = 90;
        public static final int kDpadSEAngle = 135;
        public static final int kDpadSAngle = 180;
        public static final int kDpadSWAngle = 225;
        public static final int kDpadWAngle = 270;
        public static final int kDpadNWAngle = 315;
    }

    public static final class HangConstants{
        public static final double maxExtension = 131;
    }

    public static final class ShooterConstants{
        public static final double lowSpeed = 37;
        public static final double highSpeed = 87;
    }

    public static final class ArmConstants{
        // Shoulder Limits
        public static final double maxShoulderUpSpeed = 0.4; //1.0
        public static final double maxShoulderDownSpeed = 0.4; //0.5
        public static final double maxShoulderDownSpeedNitro = 0.95;
        public static final double shoulderLowerLimit = 115;
        public static final double shoulderDangerZoneThreshold = 155; //153 
        public static final double shoulderUpperLimit = 203;

        // Wrist Limits
        public static final double maxWristSpeed = 0.3; //0.3
        public static final double wristLowerLimit = 160;
        public static final double wristDangerZoneLowerLimit = 307;
        public static final double wristShoulderStopLimit = 263;
        public static final double wristUpperLimit = 338;

        // Arm Angle Offset
        public static final double shoulderOffset = 0;
        public static final double wristOffset = 0;
        // Floor Pickup
        public static final double floorWristAngle = 313;
        public static final double floorShoulderAngle = 121.5;
        // Amp Scoring
        public static final double ampWristAngle = 288.5;
        public static final double ampShoulderAngle = 184.3;
        // Human Player Pickup
        public static final double humanWristAngle = 310;
        public static final double humanShoulderAngle = 180.3;
        // trap scoring
        public static final double trapWristAngle = 373;
        public static final double trapShoulderAngle = 187.5;
        // default
        public static final double defaultWristAngle = 197;
        public static final double defaultShoulderAngle = 160.5;
        // autonomous starting/speaker
        public static final double speakerWristAngle = 243;
        public static final double speakerShoulderAngle = 206.4;

        // Center 2 note floor shoot
        public static final double centerFloorShootWristAngle = 322;
        public static final double centerfloorShootShoulderAngle = 125.7; 
        // Top 2 note floor shoot
        public static final double topFloorShootWristAngle = 324.8;
        public static final double topfloorShootShoulderAngle = 125.7; 

        public static class State {
            private double wristAngle;
            private double shoulderAngle;
            private String stateName;

            public State(double wristAngle, double shoulderAngle, String stateName) {
                this.wristAngle = wristAngle;
                this.shoulderAngle = shoulderAngle;
                this.stateName = stateName;
            }

            public double getWristAngle(){
                return wristAngle;
            }

            public double getShoulderAngle(){
                return shoulderAngle;
            }

            public String getStateName() {
                return stateName;
            }
        }

        public final static State floorState = new State(floorWristAngle, 116, "floor");
        public final static State ampState = new State(ampWristAngle, ampShoulderAngle, "amp");
        public final static State humanState = new State(humanWristAngle, humanShoulderAngle, "human");
        public final static State trapState = new State(trapWristAngle, trapShoulderAngle, "trap");
        public final static State defaultState = new State(defaultWristAngle, defaultShoulderAngle, "default");
        public final static State speakerState = new State(speakerWristAngle, speakerShoulderAngle, "speaker");
        public final static State centerFloorShootState = new State(centerFloorShootWristAngle, centerfloorShootShoulderAngle, "centerFloorShoot");
        public final static State topFloorShootState = new State(topFloorShootWristAngle, topfloorShootShoulderAngle, "topFloorShoot");
    }
}