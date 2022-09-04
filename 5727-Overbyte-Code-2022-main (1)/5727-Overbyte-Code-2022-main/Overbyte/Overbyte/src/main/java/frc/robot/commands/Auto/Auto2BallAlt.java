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
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2BallAlt extends SequentialCommandGroup {
  /** Creates a new Auto2BallAlt. */
  public Auto2BallAlt(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset(0.0);

    PathPlannerTrajectory movementPathBall2 = PathPlanner.loadPath("moveToBall2Alt", 8.0, 5.0);
    PathPlannerTrajectory movementPathBall3 = PathPlanner.loadPath("moveToBall3Alt", 8.0, 5.0);
    PathPlannerTrajectory movementPathExpell = PathPlanner.loadPath("moveToExpellPoint", 8.0, 5.0);

    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(-6.74, -5.53)),
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
      new IntakeAutonomouslyCommand(
        intakeSubsystem,
        storageSubsystem,
        false,
        3.0
      ),
      new ShootAutonomouslyCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 9.5),
      
      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),

      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new PPSwerveControllerCommand(
            movementPathBall3,
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
          new WaitCommand(1.25),
          new IntakeAutonomouslyCommand(
            intakeSubsystem,
            storageSubsystem,
            true,
            3.0
          )
        )
      ),


      new InstantCommand(() -> Constants.translationXController.reset()),
      new InstantCommand(() -> Constants.translationYController.reset()),
      new InstantCommand(() -> Constants.rotationController.reset(0.0)),


      new PPSwerveControllerCommand(
        movementPathExpell,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        Constants.translationYController,
        Constants.translationXController,
        Constants.rotationController,
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop()),
      new ReverseIntakeCommand(
        intakeSubsystem
      )
    );
    
  }
}
