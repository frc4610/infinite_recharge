/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class intakePivot extends CommandBase {
  private Intake pivotIntake;
  private double pivotPos;
  /**
   * Creates a new intakePivot.
   */
  public intakePivot(Intake intake, double position) {
    pivotIntake = intake;
    pivotPos = position;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.pivotEncoder() < 2 && RobotContainer.pivotDirection())
    {
      RobotContainer.togglePivot(RobotContainer.pivotEncoder()+1);
    }
    else
    {
      RobotContainer.invertPivotDirection();
      RobotContainer.togglePivot(RobotContainer.pivotEncoder()-1);
      if(RobotContainer.pivotEncoder() == 0)
      {
        RobotContainer.invertPivotDirection();
      }
    }

    
    //pivotIntake.resetPivotEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotIntake.pivotIntake(pivotPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotIntake.neutralMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivotPos <= 0)
    {
      return pivotIntake.getPivotEncoderVaule() <= pivotPos;
    }
    else
    {
      return pivotIntake.getPivotEncoderVaule() >= pivotPos;
    }
  }
}
