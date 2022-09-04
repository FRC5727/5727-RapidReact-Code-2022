// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.omegabytes.ShooterConfiguration;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
  private VisionSubsystem vision;
  private PneumaticsSubsystem pneumatics;
  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  private double distance;
  private Timer shootTimer;
  private Timer storageTimer;
  private Timer timeoutTimer;
  private Double timeoutThreshold;
  private boolean linedUp;
  private boolean takeSnapshot;

  /** Creates a new ShootCommand. */
  public ShootCommand(VisionSubsystem visionSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem){
    this(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 0.0);
  }


  public ShootCommand(VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, double distance){
    this.vision = vision;
    this.pneumatics = pneumatics;
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;
    this.distance = distance;

    shootTimer = new Timer();
    storageTimer = new Timer();
    timeoutTimer = new Timer();
    timeoutThreshold = 1.0;
    linedUp = false;
    takeSnapshot = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(shooter);
    addRequirements(storage);
    addRequirements(intake);
  }
   
  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    shootTimer.reset();
    storageTimer.reset();
    timeoutTimer.reset();

    shootTimer.start();
    storageTimer.start();
    timeoutTimer.start();

    timeoutThreshold = 1.0;
    linedUp = false;
    takeSnapshot = false;

    vision.takeSnapshot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!RobotState.isTest()){
      double x = vision.getPosition();
      ShooterConfiguration config;

      boolean hasTarget;
      if (distance == 0.0){
        config = Constants.vsConversion.getValuesFromAngle(vision.getAngle(), shooter.isHoodUp());
      }else{
        config = Constants.vsConversion.getValuesFromDistance(distance, shooter.isHoodUp());
      }

      SmartDashboard.putNumber("Distance", config.getDistance());
      hasTarget = shooter.shoot(config);

      if (Math.abs(x) >= 3.0 && !linedUp){
        storageTimer.reset();
        if(Constants.driveController.getRawButton(Constants.readyToShootButton)){
          timeoutTimer.reset();
        }

        storage.stop();
        intake.stop();
        pneumatics.start();
        
      }else{

        if (!linedUp){
          vision.resetSnapshot();
        }
        linedUp = true;

        
        if (hasTarget){
          pneumatics.stop();

          if (storage.getTopProxSensor()){
            storageTimer.reset();
            timeoutTimer.reset();
          }else{
            if (!takeSnapshot){
              vision.takeSnapshot();
              takeSnapshot = !takeSnapshot;
            }
            if (storage.getBottomProxSensor() || intake.getBeamBreakSensor()){
              timeoutTimer.reset();
            }
          }

          intake.runIntake();

          if (shootTimer.get() >= 0.75){ // TODO Tune -- was 0.25 -- or specifically wait for velocity to be in range, perhaps using getSelectedSensorVelocity?
            storage.wheelFeed();
            
            if (storageTimer.get() > 0.1){
              storage.beltFeed();
            }else{
              storage.beltReverse();
            }
          }else{
            storage.wheelStop();
          }


        }else{
          shootTimer.reset();
          storageTimer.reset();

          shooter.stop();
          storage.stop();
          intake.stop();
          pneumatics.start();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isTest()){
      shooter.stop();
      storage.stop();
      intake.stop();
      pneumatics.start();
    }

    vision.resetSnapshot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(! Constants.driveController.getRawButton(Constants.readyToShootButton) && !Constants.manipController.getRawButton(Constants.overwriteShootCloseButton) && !Constants.manipController.getRawButton(Constants.overwriteShootFarButton)){
      timeoutThreshold = 0.25;
    }

    if (RobotState.isTest()){
      timeoutThreshold = -0.1;
    }
    
    return timeoutTimer.get() > timeoutThreshold;
  }
}
