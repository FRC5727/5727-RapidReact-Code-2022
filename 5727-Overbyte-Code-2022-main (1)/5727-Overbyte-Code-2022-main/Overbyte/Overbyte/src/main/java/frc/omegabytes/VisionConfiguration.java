// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionConfiguration {

    double angle;
    double distance;

    public VisionConfiguration(double angle, double distance) {
        this.angle = angle;
        this.distance = distance;
    }
    
    public double getAngle() {
        return angle;
    }

    public double getDistance() {
        return distance;
    }
}

