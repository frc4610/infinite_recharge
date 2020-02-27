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

public class tankDrive extends CommandBase {
  private DriveBase tDrivebase;
  /**
   * Creates a new tankDrive.
   */
  public tankDrive(DriveBase drivebase) {
    tDrivebase = drivebase;
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyValueL = -RobotContainer.driver.getRawAxis(1);
    double joyValueR = -RobotContainer.driver.getRawAxis(5);
    if(RobotContainer.isSlow())
    {
      joyValueL /= 2;
      joyValueR /= 2;
    }
    tDrivebase.move(ControlMode.PercentOutput, joyValueL, joyValueR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tDrivebase.move(ControlMode.PercentOutput, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
