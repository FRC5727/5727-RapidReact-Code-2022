// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1BallAlt extends SequentialCommandGroup {
  /** Creates a new Auto1BallAlt. */
  public Auto1BallAlt(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset(0.0);

    PathPlannerTrajectory movementPathShoot = PathPlanner.loadPath("moveToShootAlt", 8.0, 5.0);
    PathPlannerTrajectory movementPathMove = PathPlanner.loadPath("moveToPointAlt", 8.0, 5.0);

    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(-6.05, -4.02)),
      new PPSwerveControllerCommand(
        movementPathShoot,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        Constants.translationYController,
        Constants.translationXController,
        Constants.rotationController,
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop()),

      new ShootAutonomouslyCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 9.5),
      
      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),

      new PPSwerveControllerCommand(
        movementPathMove,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        Constants.translationYController,
        Constants.translationXController,
        Constants.rotationController,
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop())
    );
  }
}

