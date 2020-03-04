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

public class leftencoderMovement extends CommandBase {
  private double target;
  private double P = .01;
  private double rcw;
  private encoder EncoderPair;
  private double error;
  private double setpoint;
  private DriveBase driveBase;
  /**
   * Creates a new encoderMovement.
   * 
   * @param EncoderL
   * @param EncoderR
   * 
   * 
   */
  public leftencoderMovement(DriveBase tempDrive, encoder Encoder, double distance) {
    driveBase = tempDrive;
    this.EncoderPair = Encoder;
    setpoint = distance;
    addRequirements(tempDrive);
    addRequirements(Encoder);
  }
   
  // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EncoderPair.resetencoderL();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = EncoderPair.getDistanceLeft();
    error = (setpoint - target);
    this.rcw = (P *error);
    double Lspeed = rcw;
    driveBase.move(ControlMode.PercentOutput, Lspeed, .15);
    
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
    if(Math.abs(error) <= 10){
      return true;
    }
    else{
      return false;
    }
  }
}
