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

public class intakeCells extends CommandBase {
  private Intake cellIntake;
  private double intakeSpeed;
  private boolean isAuto;
  /**
   * Creates a new intakeCells.
   * 
   * @param intake Intake to use
   * @param speed Speed to set
   * @param auto Is autonoumous? If true, stops immeditaly and doesnt set back to zero at end
   */
  public intakeCells(Intake intake, double speed, boolean auto) {
    cellIntake = intake;
    intakeSpeed = speed;
    isAuto= auto;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cellIntake.intakeCells(intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cellIntake.intakeCells(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!isAuto)
    {
    cellIntake.intakeCells(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isAuto)
    {
    return !RobotContainer.operatorYButton.get();
    }
    else
    {
      return true;
    }
  }
}
