/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.limeLight;

public class visionTarget extends CommandBase {
  private limeLight limeL;
  private double distanceToPowerPort;
  private double xValueOff;
  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL) 
  {
    limeL = plimeL;
    addRequirements(plimeL);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = limeL.getXValueOff();
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limeL.vLEDoff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
