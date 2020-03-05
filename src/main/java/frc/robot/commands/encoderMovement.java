/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;

public class encoderMovement extends CommandBase {
  private double setpoint;
  private double averageEncoder;
  private double P = .005;//max p before oscillation of period T is ku. Use .6 Ku
  private double I = .000001;//use 1.2Ku/T
  private double D = .05;//use 3KuT/40
  private double integral, derivative, priorError = 0;
  private encoder EncoderPair;
  private navX turnCorrection;
  private double Straighten;
  private double error;
  private double desiredangle;
  private DriveBase driveBase;
  private Timer timer;
  private double speedL;
  private double speedR;

  /**
   * Creates a new encoderMovement.
 * @param EncoderL 
 * @param EncoderR 
   * 
   * 
   */
  public encoderMovement(DriveBase tempDrive, encoder Encoder, navX driveCorrection, double anglewanted, double distance){
    driveBase = tempDrive;
    EncoderPair = Encoder;
    turnCorrection = driveCorrection;
    setpoint = distance;
    desiredangle = anglewanted;
    timer = new Timer();
    addRequirements(tempDrive);
    addRequirements(Encoder);
    addRequirements(driveCorrection);
  }
   

    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    priorError = 0;
    integral = 0;
    derivative = 0;
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    averageEncoder = (EncoderPair.getDistanceLeft() + EncoderPair.getDistanceRight())/2;
    Straighten = (turnCorrection.getYaw() - desiredangle) * 0.02;
    error = setpoint - averageEncoder;
    integral += (error * .02);//.02 is for time in seconds
    derivative = (error - priorError)/ .02;
    speedL = (P * error) + (I*integral) + (D*derivative);
    speedR = speedL;
    speedL -= Straighten;
    speedR += Straighten;
    driveBase.hardSet(ControlMode.PercentOutput, speedL, speedR);
    priorError = error;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error) <= 10||timer.get() >= 7){
      return true;
    }
    else{
      return false;
    }
  }
}
