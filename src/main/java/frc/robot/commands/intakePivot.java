/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class intakePivot extends CommandBase {
  private Intake pivotIntake;
  private double pivotPos;
  private boolean isGoingMiddle;
  private boolean isAuto;
  private Timer timer;
  /**
   * Creates a new intakePivot.
   */
  public intakePivot(Intake intake, double position, boolean auto) {
    pivotIntake = intake;
    timer = new Timer();
    pivotPos = position;
    isAuto = auto;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    isGoingMiddle = false;
    //pivotIntake.resetPivotEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(((RobotContainer.operatorLeftBumper.get() && RobotContainer.operatorRightBumper.get())||isGoingMiddle) && !isAuto)
    {
      pivotIntake.pivotIntake(0);
      isGoingMiddle = true;
    }
    else
    {
      pivotIntake.pivotIntake(pivotPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotIntake.neutralMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((RobotContainer.operatorLeftBumper.get() || RobotContainer.operatorRightBumper.get()) && !isAuto)
    {
      return false;
    }
    else if (isAuto && timer.get() < 1)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}
