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
  private  double P = .01;
  private double rcw;
  private encoder EncoderPair;
  private navX TurnCorrection;
  private double Straighten;
  private double error;
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
    error = setpoint - EncoderPair.getDistanceLeft();
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
    TurnCorrection.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Straighten = TurnCorrection.getYaw() * .02;
    error = setpoint - EncoderPair.getDistanceLeft();
    this.rcw = (P *error);
    double Lspeed = rcw - Straighten;
    double Rspeed = rcw + Straighten;
    driveBase.move(ControlMode.PercentOutput, Lspeed, Rspeed);
    
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
    if(error <= .5){
      return true;
    }
    else{
      return false;
    }
  }
}
