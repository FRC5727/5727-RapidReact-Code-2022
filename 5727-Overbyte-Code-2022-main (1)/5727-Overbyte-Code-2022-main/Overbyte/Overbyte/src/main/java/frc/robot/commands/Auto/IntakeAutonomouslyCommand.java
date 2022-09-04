// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class IntakeAutonomouslyCommand extends CommandBase {
  private IntakeSubsystem intake;
  private StorageSubsystem storage;
  private boolean stopAtOne;
  private double timeoutThreshold;
  private Timer timeoutTimer;
  public IntakeAutonomouslyCommand(IntakeSubsystem intake, StorageSubsystem storage, boolean stopAtOne, double timeout) {
    this.intake = intake;
    this.storage = storage;
    this.stopAtOne = stopAtOne;
    this.timeoutThreshold = timeout;

    timeoutTimer = new Timer();

  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeoutTimer.reset();
    timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!((storage.getTopProxSensor() || stopAtOne) && (storage.getBottomProxSensor() || intake.getBeamBreakSensor()))){
      if (!intake.isExtended()){
        intake.extend();
        
      }
    }else{
      if (intake.isExtended()){
        intake.retract();
      }
    }
    
    intake.runIntake();

    if (!storage.getTopProxSensor()){
      storage.wheelntake();
    }else{
      storage.wheelStop();
    }

    if (!storage.getBottomProxSensor()){
      storage.beltIntake();
    }else{
      if (storage.getTopProxSensor()){
        storage.beltStop();
      }else{
        storage.beltIntake();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    
    intake.retract();
    intake.stopIntake();
    storage.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){

    if(((storage.getTopProxSensor() || stopAtOne) && (storage.getBottomProxSensor() || intake.getBeamBreakSensor()))){
      timeoutThreshold = -0.1;
    }


    return timeoutTimer.get() >= timeoutThreshold;
  }
}
