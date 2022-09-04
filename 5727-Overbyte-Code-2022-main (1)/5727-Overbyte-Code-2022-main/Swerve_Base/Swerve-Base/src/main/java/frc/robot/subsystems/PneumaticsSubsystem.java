// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  @SuppressWarnings("unused")
  private PneumaticHub pneumaticHub;
  private DoubleSolenoid intakeSolenoids;
  private DoubleSolenoid shooterSolenoids;
  //private Solenoid motorCoolerSolenoid;


  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {
    pneumaticHub = new PneumaticHub();
    intakeSolenoids = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.intakeExtendPort, Constants.intakeRetractPort);
    shooterSolenoids = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.shooterExtendPort, Constants.shooterRetractPort);
  }

  public DoubleSolenoid getIntakeSolenoids(){
    return intakeSolenoids;
  }

  public DoubleSolenoid getShooterSolenoids(){
    return shooterSolenoids;
  }

  public PneumaticHub getHub(){
    return pneumaticHub;
  }
  @Override
  public void periodic() {
    
  }
}
