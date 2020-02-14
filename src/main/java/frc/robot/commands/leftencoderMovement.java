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

public class leftencoderMovement extends CommandBase {
  private double setpoint;
  private double target;
  private  double P = .03;
  private double rcw;
  private encoder EncoderPair;
  private navX gyro;
  private double error;
  private DriveBase driveBase;

  /**
   * Creates a new encoderMovement.
 * @param EncoderL 
 * @param EncoderR 
   * 
   * 
   */
  public leftencoderMovement(DriveBase tempDrive, encoder Encoder, navX Gyro, double distance){
    driveBase = tempDrive;
    this.EncoderPair = Encoder;
    setpoint = distance;
    gyro = Gyro;
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
    error = setpoint - target;
    this.rcw = (P *error);
    double Lspeed = rcw;
    driveBase.move(ControlMode.PercentOutput, Lspeed, 0);
    
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
    if(error <= .5){
      return true;
    }
    else{
      return false;
    }
  }
}
