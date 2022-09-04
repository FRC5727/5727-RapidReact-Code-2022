// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class IMUOffsetCalculator {
    private double xOffset;
    private double yOffset;

    public IMUOffsetCalculator(double xOffset, double yOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    public Translation2d calculateOffset(double xPosition, double yPosition, double angle){
        double newX = xPosition - ((xOffset * Math.cos(angle * Math.PI / 180)) - (yOffset * Math.sin(angle * Math.PI / 180)));
        double newY = yPosition - ((xOffset * Math.sin(angle * Math.PI / 180)) + (yOffset * Math.cos(angle * Math.PI / 180)));
        return new Translation2d(newX, newY);
    }

    public Pose2d createOffsetPose(double xPosition, double yPosition, double angle){
        return new Pose2d(calculateOffset(xPosition, yPosition, angle), Rotation2d.fromDegrees(angle));
    }

    
}
