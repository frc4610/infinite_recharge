/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.encoder;
import frc.robot.RobotContainer;

public class encoderMovement extends CommandBase {
  private static final double kDistanceRequired = 120;
  private encoder EncoderPair;
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
  public encoderMovement(DriveBase tempDrive, encoder Encoder){
    driveBase = tempDrive;
    EncoderPair = Encoder;
    addRequirements(tempDrive);
    addRequirements(Encoder);
  }
   

    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DistanceL = EncoderPair.getDistanceLeft();
    DistanceR = EncoderPair.getDistanceRight();
    SmartDashboard.putNumber("DistanceL", DistanceL);
    SmartDashboard.putNumber("DistanceR", DistanceR);
    if(DistanceL <=kDistanceRequired && DistanceR <= kDistanceRequired){
      driveBase.move(ControlMode.PercentOutput, .25, .25);
    }
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
    if(DistanceL >= kDistanceRequired||DistanceR >= kDistanceRequired){
      return true;
    }
    else{
      return false;
    }
  }
}
