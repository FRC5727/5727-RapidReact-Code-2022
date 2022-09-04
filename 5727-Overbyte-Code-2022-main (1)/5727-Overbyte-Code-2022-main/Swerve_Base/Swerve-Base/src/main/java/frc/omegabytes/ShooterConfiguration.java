// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class ShooterConfiguration {

    double distance;
    double topMotorSpeed;
    double bottomMotorSpeed;
    boolean hoodUp;

    public ShooterConfiguration(double distance, double topMotorSpeed, double bottomMotorSpeed, boolean hoodUp) {
        this.distance = distance;
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed;
        this.hoodUp = hoodUp;
    }

    public double getDistance() {
        return distance;
    }

    public double getTopMotorSpeed() {
        return topMotorSpeed;
    }

    public double getBottomMotorSpeed() {
        return bottomMotorSpeed;
    }

    public boolean isHoodUp() {
        return hoodUp;
    }
}
