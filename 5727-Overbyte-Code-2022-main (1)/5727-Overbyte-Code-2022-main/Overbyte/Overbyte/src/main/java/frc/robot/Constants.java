// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import frc.omegabytes.ShooterConfiguration;
import frc.omegabytes.VisionConfiguration;
import frc.omegabytes.VisionShooterConversion;
import frc.omegabytes.VisionWeightedAverage;

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
    
    public static int storageWheelMotorPort = 9;
    public static int storageBeltMotorPort = 14; // Talon.

    public static int topShooterMotorPort = 10;
    public static int bottomShooterMotorPort = 11;

    public static int leftClimberMotorPort = 12;
    public static int rightClimberMotorPort = 13;

    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;

    public static double fleo = Math.toRadians(-39.814453);
    public static double freo = Math.toRadians(-239.589844);
    public static double rreo = Math.toRadians(-249.697266);
    public static double rleo = Math.toRadians(-198.457031);

    public static int talonCount = 14;

    public static XboxController driveController = new XboxController(0);
    public static XboxController manipController = new XboxController(1);
   
    public static boolean useCalibrateController = false;
    public static XboxController calibrateController = useCalibrateController ? new XboxController(5) : null;
   

    // Driver controller buttons/axis
    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static int extendLiftAxis = 2;
    public static int retractLiftAxis = 3;
    public static int rotationAxis = 4;
    public static int halfSpeedButton = 1;
    public static int robotOrientedButton = 2;
    public static int readyToShootButton = 5;
    public static int intakeButton = 6;

    // Manipulator controller buttons
    public static int resetGyroButton = 1;
    public static int expelBallButton = 3;
    public static int overwriteShootCloseButton = 5;
    public static int overwriteShootFarButton = 6;

    // Calibration controller buttons
    public static int topRoughAxis = 0;
    public static int bottomRoughAxis = 1;
    public static int topFineAxis = 2;
    public static int bottomFineAxis = 3;
    public static int runShooterButton = 1;
    public static int hoodUpButton = 2;
    public static int topTuneSet = 3;
    public static int bottomTuneSet = 4;
    public static int topTuneReset = 5;
    public static int bottomTuneReset = 6;
    public static int bottomTuneRange1 = 7;
    public static int topTuneRange1 = 8;
    public static int bottomTuneRange2 = 9;
    public static int topTuneRange2 = 10;
    public static int bottomTuneRange3 = 11;
    public static int topTuneRange3 = 12;

    public static int resetLeftButton = 3;
    public static int resetRightButton = 2;

    public static int intakeExtendPort = 13;
    public static int intakeRetractPort = 15;
    public static int shooterExtendPort = 0;
    public static int shooterRetractPort = 1;

    public static int storageTopProxSensorPort = 0;
    public static int storageBottomProxSensorPort = 1;
    public static int intakeBeamBreakSensorPort = 2;

    public static double deadzone = 0.1;

    public static VisionConfiguration[] visionTable_NCCMP = {
        new VisionConfiguration(39.6, 6.0),
        new VisionConfiguration(33.41, 7.0),
        new VisionConfiguration(27.76, 8.0),
        new VisionConfiguration(23.91, 9.0),
        new VisionConfiguration(20.10, 10.0),
        new VisionConfiguration(17.04, 11.0),
        new VisionConfiguration(14.24, 12.0),
        new VisionConfiguration(11.26, 13.0),
        new VisionConfiguration(9.54, 14.0),
        new VisionConfiguration(7.27, 15.0),
        new VisionConfiguration(5.38, 16.0),
        new VisionConfiguration(4.54, 17.0),
        new VisionConfiguration(2.08, 18.0),
        new VisionConfiguration(1.07, 19.0),
        new VisionConfiguration(0.0, 20.0)
    };

    // Measured with crosshair at -0.90
    public static VisionConfiguration[] visionTable = {
        new VisionConfiguration(40.09, 6.0),
        new VisionConfiguration(33.66, 7.0),
        new VisionConfiguration(29.34, 8.0),
        new VisionConfiguration(25.83, 9.0),
        new VisionConfiguration(22.30, 10.0),
        new VisionConfiguration(19.20, 11.0),
        new VisionConfiguration(16.45, 12.0),
        new VisionConfiguration(14.18, 13.0),
        new VisionConfiguration(12.24, 14.0),
        new VisionConfiguration(10.11, 15.0),
        new VisionConfiguration(8.44, 16.0),
        new VisionConfiguration(7.20, 17.0),
        new VisionConfiguration(5.57, 18.0),
        new VisionConfiguration(4.17, 19.0),
        new VisionConfiguration(0.0, 20.0) // No target
    };
 
    // TODO Consider the value of two tables
    public static ShooterConfiguration[] shootingTable = {
        new ShooterConfiguration(6.0, 1.0, -0.22, false),
        new ShooterConfiguration(7.0, 1.0, -0.25, false),
        new ShooterConfiguration(8.0, 1.0, -0.37, false),
        new ShooterConfiguration(9.0, 1.0, -0.55, false),
        new ShooterConfiguration(10.0, 1.0, -0.65, false),
        new ShooterConfiguration(9.5, 0.63, -0.63, true),
        new ShooterConfiguration(10.0, 0.65, -0.65, true),
        new ShooterConfiguration(11.0, 0.67, -0.67, true),
        new ShooterConfiguration(12.0, 0.70, -0.70, true),
        new ShooterConfiguration(13.0, 0.73, -0.73, true),
        new ShooterConfiguration(14.0, 0.75, -0.75, true),
        new ShooterConfiguration(15.0, 0.77, -0.77, true),
        new ShooterConfiguration(16.0, 0.79, -0.79, true),
        new ShooterConfiguration(17.0, 0.80 , -0.80, true),
        new ShooterConfiguration(18.0, 0.82, -0.82, true),
        new ShooterConfiguration(19.0, 0.84, -0.84, true),
        new ShooterConfiguration(20.0, 0.87, -0.87, true)
    };

    public static VisionShooterConversion vsConversion = new VisionShooterConversion(visionTable, shootingTable, 5);
    
    public static double visionAnglePrecision = 2.5;
    public static int visionPersistTicks = 100;

    public static VisionWeightedAverage visionTarget = new VisionWeightedAverage();

    public static double closeShootDistance = 7.06;
    public static double farShootDistance = 16.2;
    
    public static double translationRateLimit = 2.5;
    public static double rotationRateLimit = 2.5;

    public static double translationFeedForward = 0.1;
    public static double rotationFeedForward = 0.1;

    public static double maxVoltage = 12.0; //For "home" testing change this value to the range of 4.0 to 8.0.
    public static double wheelBase = .572;

    public static double maxVelocity = (6380.0 / 60.0 * 
        SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);
    
    public static double maxAngularVelocity = maxVelocity /
        Math.hypot(wheelBase / 2.0, wheelBase / 2.0);
    public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularVelocity);

    public static double falconRPMToUPS = 2048.0 / 600.0;

    // Was 0.1, 0.1
    public static PIDController translationXController = new PIDController(3.0, 0, 0); //10
    public static PIDController translationYController = new PIDController(3.0, 0, 0);
    public static ProfiledPIDController rotationController = new ProfiledPIDController(4.0, 0, 0, Constants.rotationConstraints); // 1.2
    
    public static double shooterkF = 1023.0/20660.0;
    public static double shooterkP = 0.1;
    public static double shooterkI = 0.001;
    public static double shooterkD = 5.0;
    public static double shooterErrorMax = 7.5;
}
