package com.adambots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants {
    public static final String kDefaultShuffleboardTab = "debug";
    public static Field2d field = new Field2d();

    public static final String canivore = "CANivore";

    public static final class VisionConstants {
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

        public static final double kPModuleTurningController = 0.7; //PID Values for turning motors .7
        public static final double kDModuleTurningController = 0.026; // 0.026
    }

    public static final class AutoConstants {
        // PD values for auton X, Y translational movement
        public static final double kPTranslationController = 12; 
        public static final double kDTranslationController = 0.1;

        // PD values for auton rotational movement
        public static final double kPThetaController = 1.1; 
        public static final double kDThetaController = 0.01;
    }

    public static final class GamepadConstants {
        // deadzone
        public static final double kDeadZone = 0.15;

        // Primary Driver Controller Port Number.
        public static final int kPrimaryDriver = RobotMap.kPrimaryControllerPort;

        // Secondary Driver Controller Port Number.
        public static final int kSecondaryDriver = RobotMap.kSecondaryControllerPort;
        /**
         * XBOX 360 South Face Button
         */
        public static final int kButtonA = 1;
        /**
         * XBOX 360 East Face Button
         */
        public static final int kButtonB = 2;
        /**
         * XBOX 360 West Face Button
         */
        public static final int kButtonX = 3;
        /**
         * XBOX 360 North Face Button
         */
        public static final int kButtonY = 4;
        /**
         * XBOX 360 Left Bumper (Top)
         */
        public static final int kButtonLB = 5;
        /**
         * XBOX 360 Right Bumper (Top)
         */
        public static final int kButtonRB = 6;
        /**
         * XBOX 360 Back Button
         */
        public static final int kButtonBack = 7;
        /**
         * XBOX 360 Start Button
         */
        public static final int kButtonStart = 8;
        /**
         * XBOX 360 Left Stick Click Button
         */
        public static final int kButtonLeftStick = 9;
        /**
         * XBOX 360 Right Stick Click Button
         */
        public static final int kButtonRightStick = 10;
    
        /**
         * XBOX 360 Left Horizontal Axis (Left=-1, Right=1)
         */
        public static final int kAxisLeftX = 0;
        /**
         * XBOX 360 Left Vertical Axis (Up=1, Down=-1)
         */
        public static final int kAxisLeftY = 1;
        /**
         * XBOX 360 Trigger Axis (LEFT)
         */
        public static final int kLeftAxisTriggers = 2;
        /**
         * XBOX 360 Trigger Axis (RIGHT)
         */
        public static final int kRightAxisTriggers = 3;
        /**
         * XBOX 360 Right Horizontal Axis (Left=-1, Right=1)
         */
        public static final int kAxisRightX = 4;
        /**
         * XBOX 360 Right Vertical Axis (Up=1, Down=-1)
         */
        public static final int kAxisRightY = 5;
    
        // the ID/port for the whole DPad
        // POV returns an angle in degrees 0-315 at 45 intervals
        public static final int kAxisDpadPov = 0;
    
        public static final int kDpadNAngle = 0;
        public static final int kDpadNEAngle = 45;
        public static final int kDpadEAngle = 90;
        public static final int kDpadSEAngle = 135;
        public static final int kDpadSAngle = 180;
        public static final int kDpadSWAngle = 225;
        public static final int kDpadWAngle = 270;
        public static final int kDpadNWAngle = 315;
    }

    public static final class ArmConstants{
        // Arm Angle Offset
        public static final double shoulderOffset = 0;
        public static final double wristOffset = 0;
        // Floor Pickup
        public static final double floorWristAngle = 276.3 % 360;
        public static final double floorShoulderAngle = 118.5 % 360;
        // Amp Scoring
        public static final double ampWristAngle = (floorWristAngle - 28) % 360; //237.8
        public static final double ampShoulderAngle = (floorShoulderAngle + 67.0) % 360; //181.6
        // Human Player Pickup
        public static final double humanWristAngle = (floorWristAngle - 0.4) % 360;
        public static final double humanShoulderAngle = (floorShoulderAngle + 66.4) % 360;
        // trap scoring
        public static final double trapWristAngle = (floorWristAngle + 60) % 360;
        public static final double trapShoulderAngle = (floorShoulderAngle + 66) % 360;
        // default
        public static final double defaultWristAngle = (floorWristAngle - 106) % 360;
        public static final double defaultShoulderAngle = (floorShoulderAngle + 46) % 360;
        // autonomous starting/speaker
        public static final double speakerWristAngle = (floorWristAngle - 63) % 360; //210.7
        public static final double speakerShoulderAngle = (floorShoulderAngle + 84.9) % 360; //199.5

        public static final double liftNoteWristAngle = (floorWristAngle - 6) % 360;
        public static final double liftNoteShoulderAngle = (floorShoulderAngle + 50) % 360; 

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
        public final static State floorState = new State(floorWristAngle,floorShoulderAngle, "floor");
        public final static State ampState = new State(ampWristAngle,ampShoulderAngle, "amp");
        public final static State humanState = new State(humanWristAngle, humanShoulderAngle, "human");
        public final static State trapState = new State(trapWristAngle, trapShoulderAngle, "trap");
        public final static State defaultState = new State(defaultWristAngle, defaultShoulderAngle, "default");
        public final static State speakerState = new State(speakerWristAngle, speakerShoulderAngle, "speaker");
        public final static State liftNoteState = new State(liftNoteWristAngle, liftNoteShoulderAngle, "liftNote");
    }
}