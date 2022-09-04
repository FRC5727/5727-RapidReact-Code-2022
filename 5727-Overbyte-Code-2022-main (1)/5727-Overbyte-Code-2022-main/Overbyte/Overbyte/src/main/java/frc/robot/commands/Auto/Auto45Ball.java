// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto45Ball extends SequentialCommandGroup {
  PathPlannerTrajectory movementPathBall = PathPlanner.loadPath("moveToBall4", 8.0, 5.0);
  PathPlannerTrajectory movementPathShoot = PathPlanner.loadPath("moveToShoot4Ball", 8.0, 5.0);

  public Auto45Ball(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new Auto3Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),

      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          movementPathBall,
          driveSubsystem::getPose,
          driveSubsystem.getKinematics(),
          Constants.translationYController,
          Constants.translationXController,
          Constants.rotationController,
          driveSubsystem::setModuleStates,
          driveSubsystem
        ),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new IntakeAutonomouslyCommand(
            intakeSubsystem,
            storageSubsystem,
            false,
            3.0
          )
        )
      ),
      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),
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
      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem, 
        intakeSubsystem,
        10.7,
        false
      )
    );
  }
}
