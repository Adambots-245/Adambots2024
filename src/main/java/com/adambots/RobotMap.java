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
import com.adambots.actuators.BaseMotor;
import com.adambots.actuators.BaseSolenoid;
import com.adambots.actuators.ElectricalSolenoid;
import com.adambots.actuators.TalonFXMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.sensors.BaseGyro;
import com.adambots.sensors.BaseProximitySensor;
import com.adambots.sensors.Gyro;
import com.adambots.sensors.PhotoEye;
import com.adambots.sensors.ThroughBoreEncoder;
import com.adambots.subsystems.SwerveModule;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * Define all the devices here
 */
public class RobotMap {
    // Robot Device Ports - PDP should be on port 1, and if pnematics are present, PCM must go on port 0
    public static final int kPDMPort = 1;
    public static final int kGyroPort = 0;
    public static final int kCANdlePort = 31;

    // Drive Ports
    public static final int kFrontRightTurningEncoderPort = 5;
    public static final int kFrontRightTurningMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 11;

    public static final int kRearRightTurningEncoderPort = 3;
    public static final int kRearRightTurningMotorPort = 18;
    public static final int kRearRightDriveMotorPort = 16;
    
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kRearLeftTurningMotorPort = 14;
    public static final int kRearLeftDriveMotorPort = 12;

    public static final int kFrontLeftTurningEncoderPort = 4;
    public static final int kFrontLeftTurningMotorPort = 15;
    public static final int kFrontLeftDriveMotorPort = 17;  

    // Arm Ports
    public static final int shoulderMotorPort = 22;
    public static final int wristMotorPort = 21;
    public static final int shoulderEncoderPort = 0;
    public static final int wristEncoderPort = 1;

    // Shooter Ports
    public static final int shooterWheelPort = 5;

    // Intake Ports
    public static final int groundIntakeMotorPort = 6;
    public static final int secondPieceInRobotEyePort = 8;
    public static final int firstPieceInRobotEyePort = 6;

    // Hang Ports
    public static final int leftHangMotorPort = 14;
    public static final int rightHangMotorPort = 10;
    public static final int leftRelayPort = 1;
    public static final int rightRelayPort = 0;
    
    // Operator Interface Ports (Joystick and XBoxControllers)
    public static final int kJoystickControllerPort = 0;
    public static final int kXboxControllerPort = 1;


    //Robot Devices
    public static final PowerDistribution PDM = new PowerDistribution(kPDMPort, ModuleType.kRev);
    public static final BaseGyro gyro = new Gyro(kGyroPort);
    public static final CANdle candleLEDs = new CANdle(kCANdlePort);

    // Arm Devices
    public static final BaseMotor shoulderMotor = new TalonFXMotor(shoulderMotorPort, true, 45.0);
    public static final BaseMotor wristMotor = new TalonFXMotor(wristMotorPort, true, 35.0);
    public static final BaseAbsoluteEncoder shoulderEncoder = new ThroughBoreEncoder(shoulderEncoderPort);
    public static final BaseAbsoluteEncoder wristEncoder = new ThroughBoreEncoder(wristEncoderPort);

    // Shooter Devices
    public static final BaseMotor shooterWheel = new TalonFXMotor(shooterWheelPort, true, 45);

    // Intake Devices
    public static final BaseMotor groundIntakeMotor = new TalonFXMotor(groundIntakeMotorPort, true, 45.0);
    public static final BaseProximitySensor secondPieceInRobotEye = new PhotoEye(secondPieceInRobotEyePort, false);
    public static final BaseProximitySensor firstPieceInRobotEye = new PhotoEye(firstPieceInRobotEyePort, false);

    // Hang Devices
    public static final BaseMotor leftHangMotor = new TalonFXMotor(leftHangMotorPort, false, 35);
    public static final BaseMotor rightHangMotor = new TalonFXMotor(rightHangMotorPort, false, 35);
    public static final BaseSolenoid leftHangSolenoid = new ElectricalSolenoid(leftRelayPort);
    public static final BaseSolenoid rightHangSolenoid = new ElectricalSolenoid(rightRelayPort);

    
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
