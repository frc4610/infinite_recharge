/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class climb extends CommandBase {
  private Climber climber;
  /**
   * Creates a new climb.
   */
  public climb(Climber tClimb) {
    climber = tClimb;
    addRequirements(tClimb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double trigValueL = RobotContainer.operator.getRawAxis(2);
    double trigValueR = RobotContainer.operator.getRawAxis(3);
    if(Math.abs(trigValueL) > .02)
    {
      climber.set(trigValueL);
    }
    else if(Math.abs(trigValueR) > .02)
    {
      climber.set(-trigValueR);
    }
    else
    {
      climber.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
