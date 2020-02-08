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
  double P = .005;
  double I = 0;
  double D = 0;
  double integral, previous_error = 0;
  double setpoint;
  navX gyro;
  DriveBase driveBase;
  private double rcw;

  /**
   * Creates a new navXTurn.
   */
  public navXTurn(navX gyro, DriveBase tempdrive, double Setpoint){
    this.gyro = (navX) gyro;
    driveBase = tempdrive;
    setpoint = Setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.resetGyro();
  }

  public void setSetpoint(int setpoint) {
    this.setpoint = setpoint;
  }

  public void PID() {
    double error = setpoint - Math.abs(gyro.getYaw()); // Error = Target - Actual
    this.integral += (error * .01);
    double derivative = (error - this.previous_error);
    this.rcw = (P * error) + (I * this.integral) + (D * derivative);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PID();
    if(setpoint > 0){
    driveBase.move(ControlMode.PercentOutput, rcw, -rcw);
    }

    else if(setpoint < 0){
      driveBase.move(ControlMode.PercentOutput, -rcw, rcw);
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
    if(Math.abs(gyro.getAngle()) >= Math.abs(setpoint)){
      return true;
    }
    else
    {
      return false;
    }
  }
}
