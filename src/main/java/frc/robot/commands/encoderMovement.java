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
  private  double P = .005;
  private double rcw;
  private encoder EncoderPair;
  private navX TurnCorrection;
  private double Straighten;
  private double error;
  private double desiredangle;
  private DriveBase driveBase;
  private Timer timer;

  /**
   * Creates a new encoderMovement.
 * @param EncoderL 
 * @param EncoderR 
   * 
   * 
   */
  public encoderMovement(DriveBase tempDrive, encoder Encoder, navX driveCorrection, double anglewanted, double distance){
    driveBase = tempDrive;
    this.EncoderPair = Encoder;
    TurnCorrection = driveCorrection;
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
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    averageEncoder = (EncoderPair.getDistanceLeft() + EncoderPair.getDistanceRight())/2;
    Straighten = (navX.getYaw() - desiredangle) * 0.02;
    error = setpoint - averageEncoder;
    this.rcw = (P *error);
    double Lspeed = (rcw - Straighten)/2;
    double Rspeed = (rcw + Straighten)/2;
    driveBase.move(ControlMode.PercentOutput, Lspeed, Rspeed);
    
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
