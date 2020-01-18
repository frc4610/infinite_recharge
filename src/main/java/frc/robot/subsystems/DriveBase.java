/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveBase extends SubsystemBase {
  private VictorSPX frontLeftDrive;
  private VictorSPX frontRightDrive;
  private VictorSPX backLeftDrive;
  private VictorSPX backRightDrive;
  private double peak;
  /**
   * Creates a new DriveBase.
   * 
   */
  public DriveBase() 
  {
    peak = 1;
    frontLeftDrive = new VictorSPX(9);
    frontRightDrive = new VictorSPX(2);
    backLeftDrive = new VictorSPX(0);
    backRightDrive = new VictorSPX(4);
    backLeftDrive.follow(frontLeftDrive);
    backRightDrive.follow(frontRightDrive);
    frontRightDrive.setInverted(true);
    backRightDrive.setInverted(true);
    RobotContainer.initMotor(frontLeftDrive, peak);
    RobotContainer.initMotor(frontRightDrive, peak);
    RobotContainer.initMotor(backLeftDrive, peak);
    RobotContainer.initMotor(backRightDrive, peak);
  }

  /**
   * 
   * @param mode Control mode to control drivebase
   * @param speedL Input, usually speed, of left side of drivebase
   * @param speedR Input, usually speed, of right side of drivebase
   */
  public void move(ControlMode mode , double speedL, double speedR){
    frontLeftDrive.set(mode, speedL);
    frontRightDrive.set(mode, speedR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
