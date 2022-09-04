// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionWeightedAverage {
    private int valueCount;
    private double[] xValues;
    private double[] yValues;
    private double[] weights;

    
    private double xOut;
    private double yOut;
    
    public VisionWeightedAverage() {
        this(10, new double[]{25.0, 20.0, 15.0, 10.0, 7.5, 7.5, 5.0, 5.0, 2.5, 2.5});
    }

    public VisionWeightedAverage(int valueCount, double[] weights) {
        this.valueCount = valueCount;
        this.weights = weights;
        clearValues();
    }

    public void clearValues(){
        xValues = new double[valueCount];
        yValues = new double[valueCount];
        for (int i = 0; i < valueCount; i++){
            xValues[i] = 0.0;
            yValues[i] = 0.0;
        }
        xOut = 0.0;
        yOut = 0.0;
    }



    public void update(double position, double angle){
        for (int i = valueCount - 1; i > 0; i--){
            xValues[i] = xValues[i - 1];
            yValues[i] = yValues[i - 1];
        }

        xValues[0] = position;
        yValues[0] = angle;

        xOut = calculateX();
        yOut = calculateY();
    }

    public double getX(){
        //return xOut;
        return xValues[0];
    }

    public double getY(){
        // TODO Temporarily disabled for testing
        //return yOut;
        return yValues[0];
    }

    private double calculateX(){
        double value = 0.0;
        double weight = 0.0;

        for (int i = 0; i < valueCount; i++){
            if (xValues[i] != 0.0){
                value += xValues[i] * weights[i];
                weight += weights[i];
            }
        }

        return value / weight;
    }

    private double calculateY(){
        double value = 0.0;
        double weight = 0.0;

        for (int i = 0; i < valueCount; i++){
            if (yValues[i] != 0.0){
                value += yValues[i] * weights[i];
                weight += weights[i];
            }
        }

        return value / weight;
    }

}
