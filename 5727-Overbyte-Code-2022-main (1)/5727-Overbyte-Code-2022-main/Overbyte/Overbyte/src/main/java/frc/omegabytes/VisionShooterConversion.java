// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionShooterConversion {
    private VisionConfiguration[] visionTable;
    private ShooterConfiguration[] shooterTable;
    private int hoodChangePosition = -1;

    public VisionShooterConversion(VisionConfiguration[] visionTable, ShooterConfiguration[] shooterTable, int hoodChangePosition) {
        this.visionTable = visionTable;
        this.shooterTable = shooterTable;

        // TODO Test this code to automatically determine the value of hoodChangePosition
//        for (int i = 1; i < shooterTable.length; i++) {
//            if (shooterTable[i].isHoodUp() != shooterTable[0].isHoodUp()) {
//                this.hoodChangePosition = i - 1;
//            }
//        }
        this.hoodChangePosition = hoodChangePosition - 1;
//        System.out.println("Hood change position is " + this.hoodChangePosition);
        assert(this.hoodChangePosition >= 0);
    }
    
    public ShooterConfiguration getValuesFromAngle(double angle, boolean isHoodUp){
        double distance = getDistanceFromAngle(angle);
        //System.out.println("DEBUG: Angle " + String.format("%.2f", angle) + " ==> distance " + String.format("%.2f", distance));
        int index = getDistanceIndex(distance, isHoodUp);
        double topMotor = getTopMotor(index, distance);
        double bottomMotor = getBottomMotor(index, distance);
        boolean setHoodUp = getHood(index);

        if (index == -1){
            //System.out.println("Robot cannot shoot from this position");
            topMotor = 0.0;
            bottomMotor = 0.0;
            setHoodUp = isHoodUp;
        }

        return new ShooterConfiguration(distance, topMotor, bottomMotor, setHoodUp);
    }

    public ShooterConfiguration getValuesFromDistance(double distance, boolean isHoodUp){
        int index = getDistanceIndex(distance, isHoodUp);
        double topMotor = getTopMotor(index, distance);
        double bottomMotor = getBottomMotor(index, distance);
        boolean setHoodUp = getHood(index);

        if (index == -1){
            //System.out.println("Robot cannot shoot from this position");
            topMotor = 0.0;
            bottomMotor = 0.0;
            setHoodUp = isHoodUp;
        }

        return new ShooterConfiguration(distance, topMotor, bottomMotor, setHoodUp);
    }

    private boolean getHood(int index){
        
        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length){
            index = shooterTable.length - 1;
        }
        
        return shooterTable[index].isHoodUp();
    }

    private double getBottomMotor(int index, double distance){
        double bottomMotor = 0.0;
        
        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length){
            index = shooterTable.length - 1;
        }

        double speedDifference = shooterTable[index].getBottomMotorSpeed() - shooterTable[index - 1].getBottomMotorSpeed();

        double distancePosition = distance - shooterTable[index - 1].getDistance();
        double distanceDifference = shooterTable[index].getDistance() - shooterTable[index - 1].getDistance();

        double distancePercentage = distancePosition / distanceDifference;
        bottomMotor = shooterTable[index - 1].getBottomMotorSpeed() + (speedDifference * distancePercentage);

        return bottomMotor;
    }

    private double getTopMotor(int index, double distance){
        double topMotor = 0.0;

        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length) {
            index = shooterTable.length - 1;
        }
        
        double speedDifference = shooterTable[index].getTopMotorSpeed() - shooterTable[index - 1].getTopMotorSpeed();

        double distancePosition = distance - shooterTable[index - 1].getDistance();
        double distanceDifference = shooterTable[index].getDistance() - shooterTable[index - 1].getDistance();

        double distancePercentage = distancePosition / distanceDifference;
        topMotor = shooterTable[index - 1].getTopMotorSpeed() + (speedDifference * distancePercentage);

        return topMotor;
    }
 
    // TODO Why not just return configuration directly?
    private int getDistanceIndex(double distance, boolean isHoodUp){
        int index = -1;
        boolean hoodUp;

        if (distance != 0.0){
            // Determine if the hood position will need to change, but prefer that it does not
            if (isHoodUp){
                if (distance < shooterTable[hoodChangePosition + 1].getDistance()){
                    hoodUp = false;
                }else{
                    hoodUp = true;
                }
            }else{
                if (distance > shooterTable[hoodChangePosition].getDistance()){
                    hoodUp = true;
                }else{
                    hoodUp = false;
                }
            }

            // Search through the appropriate configs (hood up or down) for the first entry that is beyond our shooting point
            if (hoodUp){
                for (int i = hoodChangePosition + 1; i < shooterTable.length; i++){
                    if (distance < shooterTable[i].getDistance()){
                        index = i;
                        break;
                    }
                }
            }else{
                for (int i = 1; i <= hoodChangePosition; i++){
                    if (distance < shooterTable[i].getDistance()){
                        index = i;
                        break;
                    }
                }
            }
        }

        //System.out.println("Shooting table index = " + index);
        return index;
    }

    private double getDistanceFromAngle(double angle){
        double distance = 0.0;
        for (int i = 1; i < visionTable.length; i++){
            if (visionTable[i].getAngle() < angle){
                double limelightAngleDifference = angle - visionTable[i].getAngle();
                double angleDifference = visionTable[i - 1].getAngle() - visionTable[i].getAngle();
                double distanceDifference = visionTable[i - 1].getDistance() - visionTable[i].getDistance();

                double anglePercentage = limelightAngleDifference / angleDifference;
                distance = visionTable[i].getDistance() + (distanceDifference * anglePercentage);

                break;
            }
        }

        //System.out.println(distance);
        return distance;
    }
}