/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;

public class leftencoderMovement extends CommandBase {
  private double target;
  private double P = .005;
  private double rcw;
  private navX gyro;
  private encoder EncoderPair;
  private double error;
  private double setpoint;
  private DriveBase driveBase;
  /**
   * Creates a new encoderMovement.
   * 
   * 
   * 
   * 
   */
  public leftencoderMovement(DriveBase tempDrive, navX gyroscope, double angle) {
    driveBase = tempDrive;
    gyro = gyroscope;
    setpoint = angle;
    EncoderPair = RobotContainer.mainEncoders;
    addRequirements(tempDrive);
    addRequirements(gyroscope);
  }
   
  // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = gyro.getYaw();
    error = (setpoint - target);
    this.rcw = (P *error);
    double Lspeed = rcw;
    driveBase.move(ControlMode.PercentOutput, Lspeed + 0.025, (Lspeed/2.25) + 0.025);
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
    if(Math.abs(error) <= 4){
      return true;
    }
    else{
      return false;
    }
  }
}
