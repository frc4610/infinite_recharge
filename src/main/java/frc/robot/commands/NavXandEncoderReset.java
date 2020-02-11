/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;

public class NavXandEncoderReset extends CommandBase {
  navX gyro;
  encoder EncoderPair;
  /**
   * Creates a new NavXandEncoderReset.
   */
  public NavXandEncoderReset(navX gyroscope, encoder encoders) {
    gyro = gyroscope;
    EncoderPair = encoders;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Gyro value = " + gyro.getYaw());
    System.out.println("Left Encoder value = " + EncoderPair.getDistanceLeft());
    System.out.println("Right Encoder value = " + EncoderPair.getDistanceRight());
    gyro.resetGyro();
    EncoderPair.resetencoderL();
    EncoderPair.resetencoderR();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
