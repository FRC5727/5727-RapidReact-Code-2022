// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Translation2d;
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

public class Auto5BallNew extends SequentialCommandGroup {
  PathPlannerTrajectory movementPathBall = PathPlanner.loadPath("moveToBall5", 2.0, 1.0);
  Translation2d startXY = movementPathBall.getInitialPose().getTranslation();
  PathPlannerTrajectory movementPathShoot = PathPlanner.loadPath("moveToShoot5Ball", 3.0, 1.5);

  public Auto5BallNew(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new Auto3BallNew(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
      new InstantCommand(() -> driveSubsystem.resetPose(-startXY.getX(), -startXY.getY())),

      new ParallelCommandGroup(
        new SequentialCommandGroup(
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
          new InstantCommand(() -> driveSubsystem.stop())
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
      )/*,
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
      )*/
    );
  }
}
