/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveBase extends SubsystemBase {
  private VictorSPX leftVictor;
  private VictorSPX rightVictor;
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
    leftVictor = new VictorSPX(11);
    rightVictor = new VictorSPX(3);
    leftTalon = new TalonSRX(6);
    rightTalon = new TalonSRX(4);
    leftTalon.follow(leftVictor);
    rightTalon.follow(rightVictor);
    leftVictor.setInverted(true);
    leftTalon.setInverted(true);
    RobotContainer.initMotor(leftVictor, peak);
    RobotContainer.initMotor(rightVictor, peak);
    RobotContainer.initMotor(leftTalon, peak);
    RobotContainer.initMotor(rightTalon, peak);
  }

  /**
   * Sets the drivebase to move
   * @param mode Control mode to control drivebase
   * @param speedL Input, usually speed, of left side of drivebase
   * @param speedR Input, usually speed, of right side of drivebase
   */
  public void move(ControlMode mode , double speedL, double speedR){
    leftVictor.set(mode, speedL);
    rightVictor.set(mode, speedR);
  }

  /**
   * Stops the drivebase entirely
   */
  public void stopDrivebase()
  {
    leftVictor.neutralOutput();
    rightVictor.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}