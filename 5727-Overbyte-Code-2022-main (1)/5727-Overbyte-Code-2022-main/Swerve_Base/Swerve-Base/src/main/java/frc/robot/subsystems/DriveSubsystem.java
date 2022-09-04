// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private boolean canDrive = true;
  private static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(Constants.wheelBase / 2.0, Constants.wheelBase / 2.0);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          new Translation2d(Constants.wheelBase / 2.0, Constants.wheelBase / 2.0),
          new Translation2d(Constants.wheelBase / 2.0, -Constants.wheelBase / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, Constants.wheelBase / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, -Constants.wheelBase / 2.0)
  );
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getAngle()));  // 0.0 <--- gyro.getAngle()

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveModule flm;
  private SwerveModule frm;
  private SwerveModule rrm;
  private SwerveModule rlm;
  
  /** Creates a new DriveSubSystem. */
  public DriveSubsystem() {

    // Initialize motors/cancoders to prepare them for Swerve module creation.
    // Naming is abbreviated to avoid long variable nammes. fl = Front Left, fr = Front Right,
    // rl = Rear Left, rr = Rear Right, sm = Steer Motor, dm = Drive Motor, e = Encoder,
    // eo = Encoder Offset, m = Swerve Module.
    
    flm = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L2, Constants.fldmPort, Constants.flsmPort, Constants.flePort, Constants.fleo);
    frm = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L2, Constants.frdmPort, Constants.frsmPort, Constants.frePort, Constants.freo);
    rlm = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L2, Constants.rldmPort, Constants.rlsmPort, Constants.rlePort, Constants.rleo);
    rrm = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L2, Constants.rrdmPort, Constants.rrsmPort, Constants.rrePort, Constants.rreo);


    /*for (int i = 0; i < 8; i++){
      TalonFX motor = new TalonFX(i);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    }*/
  }

  public void zeroGyroscope() {
    odometry.resetPosition(new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(gyro.getAngle())); //0.0 <--- gyro.getAngle()
  }

  public Rotation2d getRotation() {
      return odometry.getPoseMeters().getRotation();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
      this.chassisSpeeds = chassisSpeeds;
  }

  public void start(){
    canDrive = true;
  }

  public void stop(){
    canDrive = false;
  }

  @Override
  public void periodic() {
    if (canDrive){
      odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), //0.0 <--- gyro.getAngle()
              new SwerveModuleState(flm.getDriveVelocity(), new Rotation2d(flm.getSteerAngle())),
              new SwerveModuleState(frm.getDriveVelocity(), new Rotation2d(frm.getSteerAngle())),
              new SwerveModuleState(rlm.getDriveVelocity(), new Rotation2d(rlm.getSteerAngle())),
              new SwerveModuleState(rrm.getDriveVelocity(), new Rotation2d(rrm.getSteerAngle()))
      );

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

      flm.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      frm.set(-states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      rlm.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      rrm.set(-states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
  }
}
