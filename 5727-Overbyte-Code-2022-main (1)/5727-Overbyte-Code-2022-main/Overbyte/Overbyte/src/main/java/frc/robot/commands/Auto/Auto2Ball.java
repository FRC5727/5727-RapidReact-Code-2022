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



public class Auto2Ball extends SequentialCommandGroup {
  public Auto2Ball(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset(0.0);

    PathPlannerTrajectory movementPath = PathPlanner.loadPath("moveToBall2", 8.0, 3.0);

    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(-7.65, -1.88)),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
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
          new InstantCommand(() -> driveSubsystem.stop())
        ),
        new IntakeAutonomouslyCommand(
          intakeSubsystem,
          storageSubsystem,
          false,
          3.0
        )
      )
    );
  }
}
