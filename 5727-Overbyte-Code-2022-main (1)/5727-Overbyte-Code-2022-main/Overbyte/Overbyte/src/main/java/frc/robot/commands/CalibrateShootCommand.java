// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.omegabytes.ShooterConfiguration;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class CalibrateShootCommand extends CommandBase {
  private PneumaticsSubsystem pneumatics;
  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  private Timer shootTimer;
  private Timer storageTimer;
  /** Creates a new ShootCommand. */

  public CalibrateShootCommand(PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake){
    this.pneumatics = pneumatics;
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;

    shootTimer = new Timer();
    storageTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(storage);
    addRequirements(intake);
  }
   
  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    shootTimer.reset();
    storageTimer.reset();
    shootTimer.start();
    storageTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    shooter.shoot(new ShooterConfiguration(9, shooter.getTopValue(), shooter.getBottomValue(), Constants.calibrateController.getRawButton(Constants.hoodUpButton)));

    pneumatics.stop();

    if (storage.getTopProxSensor()){
      storageTimer.reset();
    }
    intake.extend();
    intake.runIntake();

    if (shootTimer.get() >= 0.25){
      storage.wheelFeed();
      
      if (storageTimer.get() > 0.1){
        storage.beltFeed();
      }else{
        storage.beltReverse();
      }
    }else{
      storage.wheelStop();
      storage.beltStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.stop();
      storage.stop();
      intake.stop();
      pneumatics.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return !Constants.calibrateController.getRawButton(Constants.runShooterButton);
  }
}

