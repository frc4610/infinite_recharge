/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limeLight extends SubsystemBase {
  private double xValueOff;
  private double yValueOff;
  private boolean validTarget;
  /**
   * Creates a new limeLight.
   */
  public limeLight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    yValueOff = 0;
    xValueOff = 0;
  }

  /**
   * Turns the limelight's LEDs off
   */
  public void vLEDoff()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  /**
   * Turns the limelight's LEDs on
   */
  public void vLEDon()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  /**
   * Stores tx, ty, and tv values from the limelight pipeline
   */
  public void visionStoreValues()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry tv = table.getEntry("tv");
    xValueOff = tx.getDouble(0);
    yValueOff = ty.getDouble(0);
    validTarget = tv.getDouble(0) == 1.0;
  }

  /**
   * 
   * @return Returns X value off of last recorded vision target
   */
  public double getXValueOff()
  {
    return xValueOff;
  }

   /**
   * 
   * @return Returns Y value off of last recorded vision target
   */
  public double getYValueOff()
  {
    return yValueOff;
  }

  /**
   * Does the limelight have a valid target?
   * @return Whether or not the limelight can see any valid targets
   */
  public boolean hasValidTarget()
  {
    return validTarget;
  }
  /**
   * Runs vision targeting to determine distance from target
   * 
   * @param h1 Disatnce from ground to camera's lens
   * @param h2 Distance from ground to target
   * @param a1 Angle, degrees, from ground to camera lens
   * @return Distance from the camera lens to the target
   */
  public double getDistance(double h1, double h2, double a1)
  {
    return (h2 - h1)/(Math.tan(a1+Math.toRadians(yValueOff)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
