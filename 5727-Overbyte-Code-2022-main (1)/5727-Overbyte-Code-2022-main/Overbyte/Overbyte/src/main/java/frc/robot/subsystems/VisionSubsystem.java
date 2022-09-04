// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.VisionWeightedAverage;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry stream;
  private NetworkTableEntry snapshot;

  private double lastGoodAngle = -1;
  private int lastAngleAge = 0;

  private double lastGoodPos = -1;
  private int lastPosAge = 0;

  private int printTick = 0;
  private int printTickMax = 0;

  private VisionWeightedAverage visionTarget;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    stream = table.getEntry("stream");
    snapshot = table.getEntry("snapshot");

    stream.setNumber(0);
    snapshot.setNumber(0);

    visionTarget = Constants.visionTarget;

  }

  public void updateTarget(){
    int valid = tv.getNumber(0).intValue();
    double currentVal = ty.getDouble(-1);

    if (valid == 0) {
      if (lastAngleAge < Constants.visionPersistTicks) {
        //System.out.println("Not valid angle -- using old value: " + lastGoodAngle);

        currentVal = lastGoodAngle;
        lastAngleAge++;
      } else {
        //System.out.println("Not valid angle -- using 0");

        currentVal = 0;
      }
    } else {
      //System.out.println("Valid angle: " + currentVal);

      lastGoodAngle = currentVal;
      lastAngleAge = 0;
    }

    currentVal = tx.getDouble(-1);

    if (valid == 0) {
      if (lastPosAge < Constants.visionPersistTicks) {
        //System.out.println("Not valid position -- using old value: " + lastGoodPos);
        currentVal = lastGoodPos;
        lastPosAge++;
      } else {
        //System.out.println("Not valid position -- using 0");
        currentVal = 0;
      }
    } else {
      //System.out.println("Valid position: " + currentVal);
      lastGoodPos = currentVal;
      lastPosAge = 0;
    }

    visionTarget.update(lastGoodPos, lastGoodAngle);
  }
 
  public double getAngle() {
    return visionTarget.getY();
  }

  public double getPosition() {
    return visionTarget.getX();
  }

  public void resetTarget(){
    visionTarget.clearValues();
  }

  public void takeSnapshot(){
    table.getEntry("snapshot").setNumber(1);
  }

  public void resetSnapshot(){
    table.getEntry("snapshot").setNumber(0);
  }

  @Override
  public void periodic() {
    
    updateTarget();
    // This method will be called once per scheduler run
    //read values periodically

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    int valid = tv.getNumber(-1).intValue();

    ////System.out.println(x);
    ////System.out.println(y);
    ////System.out.println(area);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightTarget", valid);

    if (printTickMax > 0) {
      printTick = (printTick + 1) % printTickMax;
      if (printTick == 0) {
        if (valid != 0) {
          System.out.println("Target at (" + x + ", " + y + ")");
        } else {
          System.out.println("No target");
        }
      }
    }
  }
}
