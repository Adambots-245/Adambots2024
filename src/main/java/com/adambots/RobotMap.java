/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.HashMap;
import java.util.Map;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.sensors.Gyro;
import com.adambots.sensors.PhotoEye;
import com.adambots.subsystems.SwerveModule;
import com.adambots.utils.BaseMotor;
import com.adambots.utils.TalonFXMotor;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Relay;

/**
 * Define all the devices here
 */
public class RobotMap {
    // PORTS Definition - This should be the only place to define all ports
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kRearRightTurningEncoderPort = 3;
    public static final int kFrontLeftTurningEncoderPort = 4;
    public static final int kFrontRightTurningEncoderPort = 5;

    public static final PowerDistribution PDM = new PowerDistribution(1, ModuleType.kRev);

    // Arm Ports
    public static final int shoulderMotorPort = 22;
    public static final int wristMotorPort = 21;
    public static final int shoulderEncoderPort = 0;
    public static final int wristEncoderPort = 1;
    
    // Arm Devices
    public static final BaseMotor shoulderMotor = new TalonFXMotor(shoulderMotorPort, Constants.canivore);
    public static final BaseMotor wristMotor = new TalonFXMotor(wristMotorPort, Constants.canivore);
    public static final DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(shoulderEncoderPort);
    public static final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(wristEncoderPort);

    // Shooter Ports
    public static final int shooterWheelPort = 5;

    // Shooter Devices
    public static final BaseMotor shooterWheel = new TalonFXMotor(shooterWheelPort, Constants.canivore);

    //Intake Ports
    public static final int groundIntakeMotorPort = 6;
    
    public static final int secondPieceInRobotEyePort = 6;
    public static final int firstPieceInRobotEyePort = 8;

    //Intake Devices
    public static final BaseMotor groundIntakeMotor = new TalonFXMotor(groundIntakeMotorPort, Constants.canivore);
    public static final PhotoEye secondPieceInRobotEye = new PhotoEye(secondPieceInRobotEyePort, false);
    public static final PhotoEye firstPieceInRobotEye = new PhotoEye(firstPieceInRobotEyePort, false);

    //Hang Ports
    public static final int leftHangMotorPort = 1;
    public static final int rightHangMotorPort = 1;
    public static final int leftRelayPort = 1;
    public static final int rightRelayPort = 0;

    //Hang Devices
    public static final TalonFX leftHangMotor = new TalonFX(leftHangMotorPort, Constants.canivore);
    public static final TalonFX rightHangMotor = new TalonFX(rightHangMotorPort, Constants.canivore);
    public static final Relay leftRelay = new Relay(leftRelayPort);
    public static final Relay rightRelay = new Relay(rightRelayPort);

    // CAN bus ports
    public static final int kRearLeftEncoderPort = 2;
    public static final int kRearRightEncoderPort = 3;
    public static final int kFrontLeftEncoderPort = 4;
    public static final int kFrontRightEncoderPort = 5;
    public static final int kFrontRightTurningMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 11;
    public static final int kRearRightTurningMotorPort = 18;
    public static final int kRearRightDriveMotorPort = 16;
    public static final int kRearLeftTurningMotorPort = 14;
    public static final int kRearLeftDriveMotorPort = 12;
    public static final int kFrontLeftTurningMotorPort = 15;
    public static final int kFrontLeftDriveMotorPort = 17;        

    // Operator Interface (Joystick and XBoxControllers)
    public static final int kJoystickControllerPort = 0;
    public static final int kPrimaryControllerPort = 1; // XBOX Controller
    public static final int kSecondaryControllerPort = 2; // XBOX Controller

    // Robot Devices and Sensors
    public static final Gyro gyro = new Gyro(0);

    // Robot Swerve Modules
    public static final HashMap<ModulePosition, SwerveModule> swerveModules = new HashMap<>(
        Map.of(
            ModulePosition.FRONT_LEFT,
            new SwerveModule(
                    ModulePosition.FRONT_LEFT,
                    RobotMap.kFrontLeftDriveMotorPort,
                    RobotMap.kFrontLeftTurningMotorPort,
                    RobotMap.kFrontLeftTurningEncoderPort,
                    DriveConstants.kFrontLeftDriveMotorReversed),
            ModulePosition.FRONT_RIGHT,
            new SwerveModule(
                    ModulePosition.FRONT_RIGHT,
                    RobotMap.kFrontRightDriveMotorPort,
                    RobotMap.kFrontRightTurningMotorPort,
                    RobotMap.kFrontRightTurningEncoderPort,
                    DriveConstants.kFrontRightDriveMotorReversed),
            ModulePosition.REAR_LEFT,
            new SwerveModule(
                    ModulePosition.REAR_LEFT,
                    RobotMap.kRearLeftDriveMotorPort,
                    RobotMap.kRearLeftTurningMotorPort,
                    RobotMap.kRearLeftTurningEncoderPort,
                    DriveConstants.kRearLeftDriveMotorReversed),
            ModulePosition.REAR_RIGHT,
            new SwerveModule(
                    ModulePosition.REAR_RIGHT,
                    RobotMap.kRearRightDriveMotorPort,
                    RobotMap.kRearRightTurningMotorPort,
                    RobotMap.kRearRightTurningEncoderPort,
                    DriveConstants.kRearRightDriveMotorReversed)       
        )
    );
}
