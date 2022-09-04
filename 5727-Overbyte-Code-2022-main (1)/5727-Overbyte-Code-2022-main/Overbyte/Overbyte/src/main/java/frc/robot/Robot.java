// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private CANCoder fle = new CANCoder(Constants.flePort);
  private CANCoder fre = new CANCoder(Constants.frePort);
  private CANCoder rre = new CANCoder(Constants.rrePort);
  private CANCoder rle = new CANCoder(Constants.rlePort);
  private int motorNumber = 0;

  private RobotContainer m_robotContainer;

  private int tick = 0;
  private int tickMax = 50 / 2;
  
  private void checkSwerveMotors() {
    if (tick == 0) {
      m_robotContainer.updateAngle();
    }
    tick = (tick + 1) % tickMax;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Constants.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    DataLogManager.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (motorNumber < Constants.talonCount){
      TalonFX motor = new TalonFX(motorNumber);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, 10);
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 100, 10);
      motorNumber++;
    }
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll(); 

  }

  @Override
  public void disabledPeriodic() {
    if (Constants.driveController.getAButton()){
      SmartDashboard.putNumber("Front Left Encoder", fle.getAbsolutePosition());
      SmartDashboard.putNumber("Front Right Encoder", fre.getAbsolutePosition());
      SmartDashboard.putNumber("Rear Right Encoder", rre.getAbsolutePosition());
      SmartDashboard.putNumber("Rear Left Encoder", rle.getAbsolutePosition());
    }
    checkSwerveMotors();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.getDriveTrain().resetPose();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    DataLogManager.log("ROBOT: Start of auto");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    DataLogManager.log("ROBOT: Start of teleop");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();



  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}