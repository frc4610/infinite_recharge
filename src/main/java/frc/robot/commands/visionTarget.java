/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.limeLight;

public class visionTarget extends CommandBase {
  private limeLight limeL;
  private double distanceToPowerPort;
  private double xValueOff;

  private DriveBase driveBase;
  private double leftSpeed;
  private double rightSpeed;

  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL, DriveBase tdriveBase) 
  {
    leftSpeed = 0;
    rightSpeed = 0;
    driveBase = tdriveBase;
    limeL = plimeL;
    addRequirements(tdriveBase);
    addRequirements(plimeL);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = limeL.getXValueOff();
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);
    leftSpeed -= Constants.kp * xValueOff;
    rightSpeed += Constants.kp * xValueOff;
    driveBase.move(ControlMode.PercentOutput , leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.startTankDrive();
    limeL.vLEDoff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!RobotContainer.driver1.get()||RobotContainer.tankOverride()|| Math.abs(xValueOff) < .05)
    {
      return true;
    }
    else
    {
      return false;
    }
    
  }
}
