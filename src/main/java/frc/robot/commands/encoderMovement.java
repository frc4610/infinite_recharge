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
import frc.robot.RobotContainer;

public class encoderMovement extends CommandBase {
  private double setpoint;
  private  double P = .0075;
  private double rcw;
  private encoder EncoderPair;
  private navX TurnCorrection;
  private double Straighten;
  private double DistanceL;
  private double DistanceR;
  private DriveBase driveBase;

  /**
   * Creates a new encoderMovement.
 * @param EncoderL 
 * @param EncoderR 
   * 
   * 
   */
  public encoderMovement(DriveBase tempDrive, encoder Encoder, navX driveCorrection, double distance){
    driveBase = tempDrive;
    this.EncoderPair = Encoder;
    TurnCorrection = driveCorrection;
    setpoint = distance;
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
    TurnCorrection.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DistanceL = EncoderPair.getDistanceLeft();
    DistanceR = EncoderPair.getDistanceRight();
    Straighten = TurnCorrection.getYaw() * .02;
    double error = setpoint - EncoderPair.getDistanceLeft();
    this.rcw = (P *error);
    double Lspeed = .75 - Straighten;
    double Rspeed = .75 + Straighten;
    driveBase.move(ControlMode.PercentOutput, rcw, -rcw);
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.move(ControlMode.PercentOutput, 0, 0);
    RobotContainer.startTankDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(DistanceL) >= setpoint && (Math.abs(DistanceR)) >= setpoint){
      return true;
    }
    else{
      return false;
    }
  }
}
