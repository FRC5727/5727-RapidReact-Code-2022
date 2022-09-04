// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CalibrateShootCommand;
import frc.robot.commands.ClimberMoveCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Auto.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {  
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(pneumaticsSubsystem.getIntakeSolenoids());
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(pneumaticsSubsystem.getShooterSolenoids());
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem, intakeSubsystem, visionSubsystem);
  private final ClimberMoveCommand climberMoveCommand = new ClimberMoveCommand(climberSubsystem);
  
  private final Auto1BallAlt auto1BallCommand = new Auto1BallAlt(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto1BallNew auto1BallNewCommand = new Auto1BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallShoot auto2BallCommand = new Auto2BallShoot(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallAlt auto2BallAlt = new Auto2BallAlt(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallNew auto2BallNew = new Auto2BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallShoot auto2BallShoot = new Auto2BallShoot(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallLeft auto2BallLeft = new Auto2BallLeft(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem, 0.0);
  private final Auto2BallLeft auto2BallLeftWait = new Auto2BallLeft(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem, 6.0);
  private final Auto3Ball auto3BallCommand = new Auto3Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto3BallNew auto3BallNew = new Auto3BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto45Ball auto45BallCommand = new Auto45Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    climberSubsystem.setDefaultCommand(climberMoveCommand);
    configureButtonBindings();

    // Send auto commands to SmartDashboard
    //chooser.setDefaultOption("4 or 5 Ball Auto", auto45BallCommand);
    //chooser.addOption("3 Ball Auto", auto3BallCommand);
    //chooser.addOption("2 Ball", auto2BallCommand);
    //chooser.addOption("2 Ball Left", auto2BallAlt);
    //chooser.addOption("1 Ball Alt", auto1BallCommand);
    chooser.setDefaultOption("Delayed 2 Ball Left", auto2BallLeftWait);
    chooser.addOption("New 2 Ball Left", auto2BallLeft);
    chooser.addOption("New 2 Ball Right", auto2BallShoot);
    chooser.addOption("New 1 Ball", auto1BallNewCommand);
    chooser.addOption("New 3 Ball", auto3BallNew);
    chooser.addOption("New 5 Ball", new Auto5BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem));
 
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(Constants.driveController, Constants.readyToShootButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem));
    new JoystickButton(Constants.driveController, Constants.intakeButton).whenPressed(new IntakeCommand(intakeSubsystem, storageSubsystem));
    new JoystickButton(Constants.driveController, Constants.halfSpeedButton).whenReleased(()->driveSubsystem.toggleHalfSpeed());
    new JoystickButton(Constants.driveController, Constants.robotOrientedButton).whenReleased(()->driveSubsystem.toggleRobotOriented());
  
    new JoystickButton(Constants.manipController, Constants.overwriteShootCloseButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, Constants.closeShootDistance));//vision
    new JoystickButton(Constants.manipController, Constants.overwriteShootFarButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, Constants.farShootDistance));
    new JoystickButton(Constants.manipController, Constants.expelBallButton).whenPressed(new ReverseIntakeCommand(intakeSubsystem));
    new JoystickButton(Constants.manipController, Constants.resetGyroButton).whenReleased(()->driveSubsystem.zeroGyroscope());

    if (Constants.useCalibrateController) {
      new JoystickButton(Constants.calibrateController, Constants.runShooterButton).whenPressed(new CalibrateShootCommand(pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem));

      new JoystickButton(Constants.calibrateController, Constants.topTuneSet).whenReleased(()->shooterSubsystem.setTopTunerValue());
      new JoystickButton(Constants.calibrateController, Constants.topTuneReset).whenReleased(()->shooterSubsystem.resetTopTunerValue());
      new JoystickButton(Constants.calibrateController, Constants.topTuneRange1).whenReleased(()->shooterSubsystem.setTopTunerRange(1000));
      new JoystickButton(Constants.calibrateController, Constants.topTuneRange2).whenReleased(()->shooterSubsystem.setTopTunerRange(100));
      new JoystickButton(Constants.calibrateController, Constants.topTuneRange3).whenReleased(()->shooterSubsystem.setTopTunerRange(10));
      
      new JoystickButton(Constants.calibrateController, Constants.bottomTuneSet).whenReleased(()->shooterSubsystem.setBottomTunerValue());
      new JoystickButton(Constants.calibrateController, Constants.bottomTuneReset).whenReleased(()->shooterSubsystem.resetBottomTunerValue());
      new JoystickButton(Constants.calibrateController, Constants.bottomTuneRange1).whenReleased(()->shooterSubsystem.setBottomTunerRange(1000));
      new JoystickButton(Constants.calibrateController, Constants.bottomTuneRange2).whenReleased(()->shooterSubsystem.setBottomTunerRange(100));
      new JoystickButton(Constants.calibrateController, Constants.bottomTuneRange3).whenReleased(()->shooterSubsystem.setBottomTunerRange(10));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public DriveSubsystem getDriveTrain(){
    return driveSubsystem;
  }

  public void updateAngle() {
    driveSubsystem.updateAngle();
  }
}
