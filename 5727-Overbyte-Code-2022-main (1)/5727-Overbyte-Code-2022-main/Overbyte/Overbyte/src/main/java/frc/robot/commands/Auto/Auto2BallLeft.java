// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class Auto2BallLeft extends SequentialCommandGroup {
  public Auto2BallLeft(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem, double delay) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset(0.0);

    PathPlannerTrajectory movementPathBall2 = PathPlanner.loadPath("moveToBall2Left", 3.0, 1.5);

    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(-6.74, -5.53)),
      new WaitCommand(delay),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new IntakeAutonomouslyCommand(
            intakeSubsystem,
            storageSubsystem,
            false,
            15.0
          ),
          new WaitCommand(15.0)
        ),
        new SequentialCommandGroup(
          new PPSwerveControllerCommand(
            movementPathBall2,
            driveSubsystem::getPose,
            driveSubsystem.getKinematics(),
            Constants.translationYController,
            Constants.translationXController,
            Constants.rotationController,
            driveSubsystem::setModuleStates,
            driveSubsystem
          ),
          new InstantCommand(() -> driveSubsystem.stop()),
          new WaitCommand(0.5)
        )
      ),
      new ShootAutonomouslyCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 9.8)
    );
  }
}
