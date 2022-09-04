// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto3BallNew extends SequentialCommandGroup {
  
  PathPlannerTrajectory movementPath = PathPlanner.loadPath("moveToBall3New", 3.0, 1.5);
  PathPlannerTrajectory movementPath2 = PathPlanner.loadPath("moveToBall3Part2", 3.0, 1.5);

  public Auto3BallNew(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      // Drive to pick up second ball
      new Auto2BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),

      // Move to shooting position in front of third ball
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

      // Shoot the two balls that have
      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem, 
        intakeSubsystem,
        7.5
      ),

      // Pickup the third ball
      new ParallelCommandGroup(
        new IntakeAutonomouslyCommand(
          intakeSubsystem,
          storageSubsystem,
          false,
          4.0
        ),
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          new PPSwerveControllerCommand(
            movementPath2,
            driveSubsystem::getPose,
            driveSubsystem.getKinematics(),
            Constants.translationYController,
            Constants.translationXController,
            Constants.rotationController,
            driveSubsystem::setModuleStates,
            driveSubsystem
          ),
          new InstantCommand(() -> driveSubsystem.stop())
        )
      ),
      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem, 
        intakeSubsystem,
        10.5,
        true
      )
    );
  }
}
