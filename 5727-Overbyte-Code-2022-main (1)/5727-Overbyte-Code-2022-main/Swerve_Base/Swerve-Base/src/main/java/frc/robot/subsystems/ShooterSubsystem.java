// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.ShooterConfiguration;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private PneumaticHub pneumaticHub;
  private DoubleSolenoid shooterSolenoids;
  private VisionSubsystem visionSubsystem;
  private StorageSubsystem storageSubsystem;
  private TalonFX topShooterMotor;
  private TalonFX bottomShooterMotor;
  private boolean canShoot;
  private Timer shootTimer;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(PneumaticHub pneumaticHub, DoubleSolenoid shooterSolenoids, VisionSubsystem visionSubsystem, StorageSubsystem storageSubsystem) {
    this.pneumaticHub = pneumaticHub;
    this.shooterSolenoids = shooterSolenoids;
    this.visionSubsystem = visionSubsystem;
    this.storageSubsystem = storageSubsystem;
    topShooterMotor = new TalonFX(Constants.topShooterMotorPort);
    bottomShooterMotor = new TalonFX(Constants.bottomShooterMotorPort);
    canShoot = true;
    shootTimer = new Timer();
  }

  @Override
  public void periodic() {
    if (canShoot){
      double compressorOffset = pneumaticHub.getCompressor() ? 1.5 : 0.0;
      if (Constants.driveController.getRawButton(Constants.readyToShootButton)){
        ShooterConfiguration shooterConfig = Constants.vsConversion.getValuesFromAngle(visionSubsystem.getAngle(), (shooterSolenoids.get() == Value.kForward) ? true : false);
        topShooterMotor.set(TalonFXControlMode.PercentOutput, shooterConfig.getTopMotorSpeed() + compressorOffset); // 1 - ((1 + Constants.topController.getRawAxis(3)) / 2));
        bottomShooterMotor.set(TalonFXControlMode.PercentOutput, shooterConfig.getBottomMotorSpeed() - compressorOffset); //-(1 - ((1 + Constants.bottomController.getRawAxis(3)) / 2)));
        shooterSolenoids.set(shooterConfig.isHoodUp() ? Value.kForward : Value.kReverse);

        if (shooterConfig.getDistance() != 0.0){
          shootTimer.start();
          if (shootTimer.get() > 0.5){
            storageSubsystem.feedShooter();
          }
        }else{
          //shootTimer.stop();
          //shootTimer.reset();
          //storageSubsystem.stop();
        }
      }else{
        topShooterMotor.set(TalonFXControlMode.PercentOutput, 0);
        bottomShooterMotor.set(TalonFXControlMode.PercentOutput, 0);
        shootTimer.stop();
        shootTimer.reset();
        storageSubsystem.stop();
      }
    }
  }
}
