// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.omegabytes.ShooterConfiguration;
import frc.omegabytes.VisionConfiguration;
import frc.omegabytes.VisionShooterConversion;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drive Train
    public static int fldmPort = 0;
    public static int flsmPort = 1;

    public static int frdmPort = 2;
    public static int frsmPort = 3;

    public static int rrdmPort = 4;
    public static int rrsmPort = 5;

    public static int rldmPort = 6;
    public static int rlsmPort = 7;

    public static int intakeMotorPort = 8;
    
    public static int storageConveyorPort = 13; // Talon.
    public static int storageWheelMotorPort = 9;

    public static int topShooterMotorPort = 10;
    public static int bottomShooterMotorPort = 11;

    public static int leftClimberMotorPort = 14;
    public static int rightClimberMotorPort = 15;

    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;
    //265.869140625
    public static double fleo = Math.toRadians(-258.66);
    public static double freo = Math.toRadians(-33.05);
    public static double rreo = Math.toRadians(-253.65);
    public static double rleo = Math.toRadians(-60.30);

    public static int talonCount = 12;

    public static double wheelBase = 14.0;


    public static XboxController driveController = new XboxController(0);
    public static XboxController manipController = new XboxController(1);
    public static XboxController topController = new XboxController(3);
    public static XboxController bottomController = new XboxController(4);
    //public static XboxController autoController = new XboxController(2); //If we run out of buttons we can use another arcade cabnet controller

    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static int rotationAxis = 4;
    public static int halfSpeedButton = 1;
    public static int lockWheelButton = 2;

    public static int intakeButton = 6;
    public static int readyToShootButton = 5;
    public static int expelBallButton = 3;
    public static int extendLiftButton = 4;
    public static int retractLiftButton = 5;
    
    public static int ball1Toggle = 6; // If we run out of buttons 1
    public static int ball2Toggle = 7; // If we run out of buttons 2
    public static int ball3Toggle = 8; // If we run out of buttons 3
    public static int ball4Toggle = 9; // If we run out of buttons 4
    public static int ball5Toggle = 10; // If we run out of buttons 5

    public static int intakeExtendPort = 6;
    public static int intakeRetractPort = 7;
    public static int shooterExtendPort = 1;
    public static int shooterRetractPort = 0;
    public static int motorCoolerPort = 4;

    public static int stopLidarPort = 0;

    public static double deadzone = 0.05;

    public static VisionConfiguration[] visionTable = {
        new VisionConfiguration(39.17, 6.0),
        new VisionConfiguration(33.26, 7.0),
        new VisionConfiguration(27.37, 8.0),
        new VisionConfiguration(23.19, 9.0),
        new VisionConfiguration(19.50, 10.0),
        new VisionConfiguration(16.20, 11.0),
        new VisionConfiguration(13.81, 12.0),
        new VisionConfiguration(11.06, 13.0),
        new VisionConfiguration(9.29, 14.0),
        new VisionConfiguration(7.30, 15.0),
        new VisionConfiguration(5.55, 16.0),
        new VisionConfiguration(3.93, 17.0),
        new VisionConfiguration(2.33, 18.0),
        new VisionConfiguration(0.75, 19.0),
        new VisionConfiguration(0.0, 20.0)
    };

    public static ShooterConfiguration[] shootingTable = {
        new ShooterConfiguration(6.0, 1.0, -0.22, false),
        new ShooterConfiguration(7.0, 1.0, -0.27, false),
        new ShooterConfiguration(8.0, 1.0, -0.37, false),
        new ShooterConfiguration(9.0, 1.0, -0.53, false),
        new ShooterConfiguration(10.0, 1.0, -0.60, false),
        new ShooterConfiguration(9.0, 0.67, -0.67, true),
        new ShooterConfiguration(10.0, 0.67, -0.67, true),
        new ShooterConfiguration(11.0, 0.69, -0.69, true),
        new ShooterConfiguration(12.0, 0.71, -0.71, true),
        new ShooterConfiguration(13.0, 0.73, -0.73, true),
        new ShooterConfiguration(14.0, 0.75, -0.75, true),
        new ShooterConfiguration(15.0, 0.77, -0.77, true),
        new ShooterConfiguration(16.0, 0.78, -0.78, true),
        new ShooterConfiguration(17.0, 0.79, -0.79, true),
        new ShooterConfiguration(18.0, 0.81, -0.81, true),
        new ShooterConfiguration(19.0, 0.84, -0.84, true),
        new ShooterConfiguration(20.0, 0.86, -0.86, true)
    };

    public static VisionShooterConversion vsConversion = new VisionShooterConversion(visionTable, shootingTable, 5);
}

