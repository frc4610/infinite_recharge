/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.navX;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class navXTurn extends CommandBase {
  private static double kTurnDesired;
  private navX gyro;
  private double yaw;
  private DriveBase driveBase;
  /**
   * Creates a new navXTurn.
   */
  public navXTurn(navX gyrNavX, DriveBase tempdrive, double kTurnWant) {
    gyro = gyrNavX;
    driveBase = tempdrive;
    kTurnDesired = kTurnWant;
    addRequirements(gyrNavX);
    addRequirements(tempdrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yaw = gyro.getYaw();
    if(yaw > 0){
      driveBase.move(ControlMode.PercentOutput, .5, -.5);
    }
    else if(yaw < 0){
      driveBase.move(ControlMode.PercentOutput, -.5, .5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.move(ControlMode.PercentOutput, 0, 0);
    gyro.resetGyro();
    RobotContainer.startTankDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(yaw >= Math.abs(kTurnDesired)){
      return true;
    }
    else
    {
    return false;
    }
  }
}
