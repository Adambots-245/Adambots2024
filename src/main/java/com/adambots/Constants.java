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

    public static final class LEDConstants {
        public static final int LEDS_IN_STRIP = 62;

        public static final Color off = new Color(0, 0, 0);
        public static final Color adambotsYellow = new Color(255, 255, 0);
        public static final Color yellow = new Color(255, 255, 0);
        public static final Color blue = new Color(0, 0, 255);
        public static final Color orange = new Color(180, 90, 5);
        public static final Color pink = new Color(255, 200, 200);
        public static final Color purple = new Color(255, 0, 255);
        public static final Color red = new Color(255, 150, 0);
        public static final Color green = new Color(0, 255, 0);
    }

    public static final class VisionConstants {
        public static final Pose2d aprilTagRedPose2d = new Pose2d(new Translation2d(1.23, 2.55), new Rotation2d());
        public static final String noteLimelite = "limelight-notebot";
        public static final String aprilLimelite = "limelight-april";

        public static final double kPThetaController = 0.3; 
        public static final double kDThetaController = 0.001;

        public static final double kPTranslateController = 3; 
        public static final double kDTranslateController = 0.01;
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
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        // Xbox controller joystick deadzone
        public static final double kDeadZone = 0.15;

        //Max speed of the robot in m/s, used in teleop and auton (should be set to real world value)
        public static final double kMaxSpeedMetersPerSecond = 4.35; 
        //Rotational speed factor in rad/s of the robot to be used for the teleop drive command
        public static final double kTeleopRotationalSpeed = 10; 
    }

    public static final class ModuleConstants {
        public static final int kDriveCurrentLimit = 32; //Current limit in amps of drive motors, higher values mean faster acceleration but lower battery life
        public static final int kTurningCurrentLimit = 21; //Current limit in amps of turning motors
        public static final double kNominalVoltage = 12.6; //Nominal battery voltage for motor voltage compensation

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
        public static final double kPTranslationController = 5; 
        public static final double kDTranslationController = 0.11;

        // PD values for auton rotational movement
        public static final double kPThetaController = 3; 
        public static final double kDThetaController = 0.01;
    }

    public static final class HangConstants{
        public static final double maxExtension = 150;
        public static final double lowExtension = 0;
    }

    public static final class IntakeConstants{
        public static final double intakeSpeed = 0.2;
        public static final double humanSpeed = 0.2;
        public static final double ampSpeed = 0.5;
        public static final double lowSpeed = 0.1;
        public static final double shootSpeed = 1;
    }

    public static final class ShooterConstants{
        public static final double idleSpeed = 30;
        public static final double lowSpeed = 65;
        public static final double mediumSpeed = 70;
        public static final double highSpeed = 80;
        public static final double maxSpeed = 94;
    }

    public static final class ArmConstants{
        // Arm Angle Offset (Is applied when states are created - affects every state)
        public static final double shoulderOffset = 0;
        public static final double wristOffset = 0;

        // Shoulder Limits
        public static final double maxShoulderUpSpeed = 0.4; //1.0
        public static final double maxShoulderDownSpeed = 0.4; //0.5
        public static final double maxShoulderDownSpeedNitro = 0.95;

        public static final double shoulderLowerLimit = 110;
        public static final double shoulderDangerZoneThreshold = 153;
        public static final double shoulderUpperLimit = 203;

        // Wrist Limits
        public static final double maxWristSpeed = 0.3;
        
        public static final double wristLowerLimit = 160;
        public static final double wristDangerZoneLowerLimit = 307;
        public static final double wristShoulderStopLimit = 280;
        public static final double wristUpperLimit = 338;
     
        // Floor Pickup
        public static final double floorWristAngle = 313;
        public static final double floorShoulderAngle = 114.5;
      
        // Amp Scoring
        public static final double ampWristAngle = 283;
        public static final double ampShoulderAngle = 184.3;
      
        // Human Player Pickup
        public static final double humanWristAngle = 310;
        public static final double humanShoulderAngle = 180.3;
      
        // trap scoring
        public static final double trapWristAngle = 373;
        public static final double trapShoulderAngle = 187.5;
     
        // default
        public static final double defaultWristAngle = 197;
        public static final double defaultShoulderAngle = 157;
     
        // autonomous starting/speaker
        public static final double speakerWristAngle = 242.3;
        public static final double speakerShoulderAngle = 206.4;

        // Center 2 note floor shoot
        public static final double centerFloorShootWristAngle = 320.75;
        public static final double centerfloorShootShoulderAngle = 125.7; 
     
        // Top 2 note floor shoot
        public static final double topFloorShootWristAngle = 321.7;
        public static final double topfloorShootShoulderAngle = 125.7; 

        //Hang
        public static final double hangWristAngle = 186;
        public static final double hangShoulderAngle = 146.5; 

        //Close Floor Shoot State
        public static final double closeFloorShootWristAngle = 312;
        public static final double closeFloorShootShoulderAngle = 123.7;

        // Bottom 2 note floor shoot **NOT TUNED**
        public static final double bottomFloorShootWristAngle = 324.8;
        public static final double bottomfloorShootShoulderAngle = 125.7; 

        // Bottom 2 note floor shoot **NOT TUNED**
        public static final double feedWristAngle = 312+wristOffset;
        public static final double feedShoulderAngle = 123.7+shoulderOffset; 

        public static class State {
            private double wristAngle;
            private double shoulderAngle;
            private StateName stateName;
            private double wristTolerance = 0.0;
            private double shoulderTolerance = 0.0;

            public State(double wristAngle, double shoulderAngle, StateName stateName) {
                this.wristAngle = wristAngle+wristOffset;
                this.shoulderAngle = shoulderAngle+shoulderOffset;
                this.stateName = stateName;
            }

            public State(double wristAngle, double shoulderAngle, StateName stateName, double wristTolerance, double shoulderTolerance) {
                this.wristAngle = wristAngle;
                this.shoulderAngle = shoulderAngle;
                this.stateName = stateName;
                this.wristTolerance = wristTolerance;
                this.shoulderTolerance = shoulderTolerance;
            }

            public double getWristAngle(){
                return wristAngle;
            }

            public double getShoulderAngle(){
                return shoulderAngle;
            }

            public StateName getStateName() {
                return stateName;
            }

            public double getWristTolerance() {
                return wristTolerance;
            }

            public double getShoulderTolerance() {
                return shoulderTolerance;
            }
        }

        public enum StateName {
            FLOOR,
            AMP,
            HUMAN,
            TRAP,
            DEFAULT,
            SPEAKER,
            TOP_FLOOR_SHOOT,
            CENTER_FLOOR_SHOOT,
            BOTTOM_FLOOR_SHOOT,
            CLOSE_FLOOR_SHOOT,
            HANG,
            CUSTOM
        }

        public final static State floorState = new State(floorWristAngle, floorShoulderAngle, StateName.FLOOR);
        public final static State ampState = new State(ampWristAngle, ampShoulderAngle, StateName.AMP, 3.5, 3.5);
        public final static State humanState = new State(humanWristAngle, humanShoulderAngle, StateName.HUMAN, 2, 2);
        public final static State trapState = new State(trapWristAngle, trapShoulderAngle, StateName.TRAP);
        public final static State defaultState = new State(defaultWristAngle, defaultShoulderAngle, StateName.DEFAULT, 5, 5);
        public final static State speakerState = new State(speakerWristAngle, speakerShoulderAngle, StateName.SPEAKER, 1, 1);
        public final static State centerFloorShootState = new State(centerFloorShootWristAngle, centerfloorShootShoulderAngle, StateName.CENTER_FLOOR_SHOOT);
        public final static State topFloorShootState = new State(topFloorShootWristAngle, topfloorShootShoulderAngle, StateName.TOP_FLOOR_SHOOT);
        public final static State bottomFloorShootState = new State(bottomFloorShootWristAngle, bottomfloorShootShoulderAngle, StateName.BOTTOM_FLOOR_SHOOT);
        public final static State closeFloorShootState = new State(closeFloorShootWristAngle, closeFloorShootShoulderAngle, StateName.CLOSE_FLOOR_SHOOT);
        public final static State hangState = new State(hangWristAngle, hangShoulderAngle, StateName.HANG);
    }
}