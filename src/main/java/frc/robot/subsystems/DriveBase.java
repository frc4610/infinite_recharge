/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveBase extends SubsystemBase {
  private TalonSRX leftLeadTalon;
  private TalonSRX righLeadTalon;
  private TalonSRX leftTalon;
  private TalonSRX rightTalon;
  private double peak;
  /**
   * Creates a new DriveBase.
   * 
   */
  public DriveBase() 
  {
    peak = 1;
    leftLeadTalon = new TalonSRX(8);
    righLeadTalon = new TalonSRX(7);
    leftTalon = new TalonSRX(9);
    rightTalon = new TalonSRX(6);
    leftTalon.follow(leftLeadTalon);
    rightTalon.follow(righLeadTalon);
    leftLeadTalon.setInverted(true);
    leftTalon.setInverted(true);
    RobotContainer.initMotor(leftLeadTalon, peak);
    RobotContainer.initMotor(righLeadTalon, peak);
    RobotContainer.initMotor(leftTalon, peak);
    RobotContainer.initMotor(rightTalon, peak);
  }

  /**
   * 
   * @param mode Control mode to control drivebase
   * @param speedL Input, usually speed, of left side of drivebase
   * @param speedR Input, usually speed, of right side of drivebase
   */
  public void move(ControlMode mode , double speedL, double speedR){
    leftLeadTalon.set(mode, speedL);
    righLeadTalon.set(mode, speedR);
  }

  /**
   * Stops the drivebase entirely
   */
  public void stopDrivebase()
  {
    leftLeadTalon.neutralOutput();
    righLeadTalon.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}