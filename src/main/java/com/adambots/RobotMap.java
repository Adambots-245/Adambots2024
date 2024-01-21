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
import com.adambots.subsystems.SwerveModule;

/**
 * Define all the devices here
 */
public class RobotMap {
    // PORTS Definition - This should be the only place to define all ports
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
    public static final int kPrimaryControllerPort = 0; // XBOX Controller - overlaps with joystick so you can use an xbox controller instead of joystick
    public static final int kSecondaryControllerPort = 1; // XBOX Controller for secondary driver

    // Robot Devices and Sensors
    public static final Gyro gyro = new Gyro();

    // Robot Swerve Modules
    public static final HashMap<ModulePosition, SwerveModule> swerveModules = new HashMap<>(
        Map.of(
            ModulePosition.FRONT_LEFT,
            new SwerveModule(
                    ModulePosition.FRONT_LEFT,
                    RobotMap.kFrontLeftDriveMotorPort,
                    RobotMap.kFrontLeftTurningMotorPort,
                    RobotMap.kFrontLeftEncoderPort,
                    DriveConstants.kFrontLeftDriveMotorReversed),
            ModulePosition.FRONT_RIGHT,
            new SwerveModule(
                    ModulePosition.FRONT_RIGHT,
                    RobotMap.kFrontRightDriveMotorPort,
                    RobotMap.kFrontRightTurningMotorPort,
                    RobotMap.kFrontRightEncoderPort,
                    DriveConstants.kFrontRightDriveMotorReversed),
            ModulePosition.REAR_LEFT,
            new SwerveModule(
                    ModulePosition.REAR_LEFT,
                    RobotMap.kRearLeftDriveMotorPort,
                    RobotMap.kRearLeftTurningMotorPort,
                    RobotMap.kRearLeftEncoderPort,
                    DriveConstants.kRearLeftDriveMotorReversed),
            ModulePosition.REAR_RIGHT,
            new SwerveModule(
                    ModulePosition.REAR_RIGHT,
                    RobotMap.kRearRightDriveMotorPort,
                    RobotMap.kRearRightTurningMotorPort,
                    RobotMap.kRearRightEncoderPort,
                    DriveConstants.kRearRightDriveMotorReversed)       
        )
    );
}
