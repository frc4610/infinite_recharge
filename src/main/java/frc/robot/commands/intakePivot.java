/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class intakePivot extends CommandBase {
  private Intake pivotIntake;
  private double pivotSpeed;
  /**
   * Creates a new intakePivot.
   */
  public intakePivot(Intake intake, double speed) {
    pivotIntake = intake;
    pivotSpeed = speed;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotIntake.pivotIntake(pivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotIntake.pivotIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //add stop when the intake pivots to the position
    return false;
  }
}
