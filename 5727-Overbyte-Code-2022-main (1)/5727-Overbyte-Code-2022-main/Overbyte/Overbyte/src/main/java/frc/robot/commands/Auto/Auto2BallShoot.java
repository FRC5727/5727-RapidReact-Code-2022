// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto2BallShoot extends SequentialCommandGroup {
  public Auto2BallShoot(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    PathPlannerTrajectory movementPath = PathPlanner.loadPath("moveToShoot2New", 3.0, 1.5);

    addCommands(
      new Auto2BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
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
      new ShootAutonomouslyCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 7.5)
    );
  }
}