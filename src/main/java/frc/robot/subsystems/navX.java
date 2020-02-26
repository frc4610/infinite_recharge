/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class navX extends SubsystemBase {
  private static AHRS gyro;

  /**
   * Creates a new navX.
   */
  public navX() {
    gyro = new AHRS(SPI.Port.kMXP);
  }

  /**
   * Resets the gyro. Best used at the beginning of a match
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Function for accessing the navx's gyro feature
   * 
   * @return Returns yaw (z-axis) of gyro in degree. Note that this is a true
   *         value, and can be above 360
   */
  public static double getYaw() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
