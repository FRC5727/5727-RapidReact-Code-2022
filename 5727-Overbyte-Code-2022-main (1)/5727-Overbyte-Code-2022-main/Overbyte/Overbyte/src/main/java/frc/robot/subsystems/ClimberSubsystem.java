// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimberMotor = new TalonFX(Constants.leftClimberMotorPort);
    rightClimberMotor = new TalonFX(Constants.rightClimberMotorPort);
  }

  public void move(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * -1.0);
    leftClimberMotor.getStatorCurrent();
    rightClimberMotor.getStatorCurrent();
  }

  public void moveLeft(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
  }

  public void moveRight(double speed){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
  }

  public void resetLeft(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, -0.15);
  }  

  public void resetRight(){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.15);
  }  

  public void stopLeft(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void stopRight(){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void stop(){
    stopLeft();
    stopRight();
  }

  @Override
  public void periodic() {
  }
}
