// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto3Ball extends SequentialCommandGroup {
  
  PathPlannerTrajectory movementPath = PathPlanner.loadPath("moveToBall3", 8.0, 5.0);

  public Auto3Ball(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new Auto2Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),

      new PPSwerveControllerCommand(
        movementPath,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        Constants.translationYController,
        Constants.translationXController,
        Constants.rotationController,
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop()),

      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem, 
        intakeSubsystem,
        9.5,
        true
      )
    );
  }
}
