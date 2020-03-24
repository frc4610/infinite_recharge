/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;

public class leftencoderMovement extends CommandBase {
  private double P = .0005;//max p before oscillation of period T is ku. Use .6 Ku
  private double I = .00000;//use 1.2Ku/T
  private double D = .0;//use 3KuT/40
  private double integral, derivative, priorError = 0;
  private encoder EncoderPair;
  private navX gyro;
  private double error;
  private double setpoint;
  private DriveBase driveBase;
  private double speedL;
  /**
   * Creates a new encoderMovement.
   * 
   * 
   * 
   * 
   */
  public leftencoderMovement(DriveBase tempDrive, encoder Encoder, navX Gyro, double desiredAngle) {
    driveBase = tempDrive;
    EncoderPair = Encoder;
    setpoint = desiredAngle;
    gyro = Gyro;
    addRequirements(tempDrive);
    addRequirements(Encoder);
    addRequirements(Gyro);
  }
   
  // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    priorError = 0;
    integral = 0;
    derivative = 0;
    EncoderPair.resetencoderL();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - gyro.getYaw();
    integral += (error * .02);//.02 is for time in seconds
    derivative = (error - priorError)/ .02;
    speedL = (P * error) + (I*integral) + (D*derivative);
    driveBase.move(ControlMode.PercentOutput, speedL, 0);
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.move(ControlMode.PercentOutput, 0, 0);
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error) <= 2){
      return true;
    }
    else{
      return false;
    }
  }
}