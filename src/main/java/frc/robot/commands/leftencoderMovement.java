/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;

public class leftencoderMovement extends CommandBase {
  private double currentangle;
  private double currentL;
  private double currentR;
  private double P = .00678;
  private double I = .0000678;
  private double D = 0;
  private double power;
  private navX gyro;
  private encoder EncoderPair;
  private double errorangle;
  private double errorL;
  private double errorR;
  private double integralangle;
  private double setpointangle;
  private double setpointL;
  private double setpointR;
  private DriveBase driveBase;
  /**
   * Creates a new encoderMovement.
   * 
   * 
   * 
   * 
   */
  public leftencoderMovement(DriveBase tempDrive, navX gyroscope, encoder Encoder, double angle, double DistanceL, double DistanceR) {
    driveBase = tempDrive;
    gyro = gyroscope;
    EncoderPair = Encoder;
    setpointangle = angle;
    setpointL = DistanceL;
    setpointR = DistanceR;
    EncoderPair = RobotContainer.mainEncoders;
    addRequirements(tempDrive);
    addRequirements(gyroscope);
    addRequirements(Encoder);
  }
   
  // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentangle = gyro.getYaw();
    currentL = EncoderPair.getDistanceLeft();
    currentR = EncoderPair.getDistanceRight();
    errorangle = (setpointangle - currentangle);
    errorL = (setpointL - currentL);
    errorR = (setpointR - currentR);
    //integralL += (errorL * .02);
    //integralR += (errorR * .02);
    integralangle += (errorangle * .02);
    this.power = (P * errorangle) + (I * integralangle);
    double speed = power;

    driveBase.move(ControlMode.PercentOutput, (speed)/1.5, (speed * .42)/1.5);
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
    if(Math.abs(errorangle) <= 2){
      return true;
    }
    else{
      return false;
    }
  }
}