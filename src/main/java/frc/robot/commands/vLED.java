/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limeLight;

public class vLED extends CommandBase {
  private limeLight limeL;
  private boolean start;
  /**
   * Creates a new vLED.
   * 
   * @param LimeL The limelight to use, should only be one
   * @param Start True if the light to be turned on, false if light should be off
   */
  public vLED(limeLight LimeL, boolean Start) {
    limeL = LimeL;
    start = Start; 
    addRequirements(LimeL);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(start)
    {
    limeL.vLEDon();
    }
    else
    {
      limeL.vLEDoff();
    }
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
