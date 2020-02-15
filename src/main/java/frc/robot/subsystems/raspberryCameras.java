/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class raspberryCameras extends SubsystemBase {
  private NetworkTable storage;
  //private NetworkTable finder;
  /**
   * Creates a new raspberryCameras.
   */
  public raspberryCameras() {
    storage = NetworkTableInstance.getDefault().getTable("GStreamer");
    //finder = NetworkTableInstance.getDefault().getTable("Finder");
  }
  
  public int cellsInStorage()
  {
    return (int) storage.getEntry("foo").getDouble(0);
  }
  public double xValueFromBest()
  {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
